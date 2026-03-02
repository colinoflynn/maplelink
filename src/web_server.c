#include "web_server.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lwip/tcp.h"
#include "lwip/timeouts.h"

#define WS_SERVER_PORT 81
#define WS_RX_BUF_LEN 2048
#define WS_FRAME_MAX_PAYLOAD 512
#define WS_TX_FRAME_BUF_LEN 1536
#define WS_HANDSHAKE_TIMEOUT_MS 10000u

typedef struct {
  uint32_t state[5];
  uint64_t count;
  uint8_t buf[64];
} sha1_ctx_t;

typedef struct {
  uint8_t opcode;
  uint16_t len;
  uint8_t payload[WS_FRAME_MAX_PAYLOAD];
} ws_frame_t;

struct ws_conn {
  struct tcp_pcb *pcb;
  bool upgraded;
  uint32_t last_activity_ms;
  uint8_t rx[WS_RX_BUF_LEN];
  int rx_len;
};

static struct tcp_pcb *g_listen;
static const ws_app_handler_t *g_handler;
static uint8_t g_ws_tx_frame[WS_TX_FRAME_BUF_LEN];

static uint32_t rol32(uint32_t x, uint32_t n) { return (x << n) | (x >> (32 - n)); }

static void sha1_transform(uint32_t st[5], const uint8_t block[64]) {
  uint32_t w[80];
  for (int i = 0; i < 16; i++) {
    w[i] = ((uint32_t)block[i * 4 + 0] << 24) |
           ((uint32_t)block[i * 4 + 1] << 16) |
           ((uint32_t)block[i * 4 + 2] << 8) |
           ((uint32_t)block[i * 4 + 3]);
  }
  for (int i = 16; i < 80; i++) w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

  uint32_t a = st[0], b = st[1], c = st[2], d = st[3], e = st[4];
  for (int i = 0; i < 80; i++) {
    uint32_t f, k;
    if (i < 20) {
      f = (b & c) | ((~b) & d);
      k = 0x5A827999;
    } else if (i < 40) {
      f = b ^ c ^ d;
      k = 0x6ED9EBA1;
    } else if (i < 60) {
      f = (b & c) | (b & d) | (c & d);
      k = 0x8F1BBCDC;
    } else {
      f = b ^ c ^ d;
      k = 0xCA62C1D6;
    }
    uint32_t temp = rol32(a, 5) + f + e + k + w[i];
    e = d;
    d = c;
    c = rol32(b, 30);
    b = a;
    a = temp;
  }
  st[0] += a;
  st[1] += b;
  st[2] += c;
  st[3] += d;
  st[4] += e;
}

static void sha1_init(sha1_ctx_t *c) {
  c->state[0] = 0x67452301;
  c->state[1] = 0xEFCDAB89;
  c->state[2] = 0x98BADCFE;
  c->state[3] = 0x10325476;
  c->state[4] = 0xC3D2E1F0;
  c->count = 0;
}

static void sha1_update(sha1_ctx_t *c, const uint8_t *data, size_t len) {
  size_t i = 0;
  size_t idx = (size_t)(c->count & 63);
  c->count += len;

  if (idx) {
    size_t take = 64 - idx;
    if (take > len) take = len;
    memcpy(c->buf + idx, data, take);
    idx += take;
    i += take;
    if (idx == 64) sha1_transform(c->state, c->buf);
  }

  for (; i + 64 <= len; i += 64) sha1_transform(c->state, data + i);
  if (i < len) memcpy(c->buf, data + i, len - i);
}

static void sha1_final(sha1_ctx_t *c, uint8_t out[20]) {
  uint8_t pad[64] = {0x80};
  uint8_t lenbuf[8];
  uint64_t bits = c->count * 8;

  for (int i = 0; i < 8; i++) lenbuf[7 - i] = (uint8_t)(bits >> (i * 8));

  size_t idx = (size_t)(c->count & 63);
  size_t padlen = (idx < 56) ? (56 - idx) : (120 - idx);
  sha1_update(c, pad, padlen);
  sha1_update(c, lenbuf, 8);

  for (int i = 0; i < 5; i++) {
    out[i * 4 + 0] = (uint8_t)(c->state[i] >> 24);
    out[i * 4 + 1] = (uint8_t)(c->state[i] >> 16);
    out[i * 4 + 2] = (uint8_t)(c->state[i] >> 8);
    out[i * 4 + 3] = (uint8_t)(c->state[i]);
  }
}

static const char b64tab[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static int base64_encode(const uint8_t *in, int inlen, char *out, int outlen) {
  int o = 0;
  for (int i = 0; i < inlen; i += 3) {
    uint32_t v = (uint32_t)in[i] << 16;
    if (i + 1 < inlen) v |= (uint32_t)in[i + 1] << 8;
    if (i + 2 < inlen) v |= (uint32_t)in[i + 2];
    int pad = (i + 2 >= inlen) ? ((i + 1 >= inlen) ? 2 : 1) : 0;

    if (o + 4 >= outlen) return -1;
    out[o++] = b64tab[(v >> 18) & 63];
    out[o++] = b64tab[(v >> 12) & 63];
    out[o++] = (pad >= 2) ? '=' : b64tab[(v >> 6) & 63];
    out[o++] = (pad >= 1) ? '=' : b64tab[v & 63];
  }

  if (o >= outlen) return -1;
  out[o] = 0;
  return o;
}

static err_t tcp_write_all(struct tcp_pcb *pcb, const void *data, u16_t len) {
  const uint8_t *p = (const uint8_t *)data;

  while (len > 0) {
    u16_t snd = tcp_sndbuf(pcb);
    u16_t chunk = len;
    err_t e;

    if (snd == 0) return ERR_MEM;
    if (chunk > snd) chunk = snd;

    e = tcp_write(pcb, p, chunk, TCP_WRITE_FLAG_COPY);
    if (e != ERR_OK) return e;

    p += chunk;
    len -= chunk;
  }

  return ERR_OK;
}

static bool ws_send(ws_conn_t *conn, uint8_t opcode, const uint8_t *payload, uint16_t plen) {
  uint8_t hdr[4];
  int hlen;
  uint16_t frame_len;
  err_t e;

  if (!conn || !conn->pcb || !conn->upgraded) return false;

  hdr[0] = 0x80 | (opcode & 0x0F);
  if (plen <= 125) {
    hdr[1] = (uint8_t)plen;
    hlen = 2;
  } else {
    hdr[1] = 126;
    hdr[2] = (uint8_t)(plen >> 8);
    hdr[3] = (uint8_t)(plen & 0xFF);
    hlen = 4;
  }

  // WebSocket frames must be written atomically relative to retries. If we
  // enqueue only part of a frame and then retry, the byte stream is corrupted.
  frame_len = (uint16_t)(hlen + plen);
  if (tcp_sndbuf(conn->pcb) < frame_len) return false;

  if (frame_len > WS_TX_FRAME_BUF_LEN) return false;
  memcpy(g_ws_tx_frame, hdr, (size_t)hlen);
  if (plen > 0 && payload) memcpy(g_ws_tx_frame + hlen, payload, (size_t)plen);

  // Enqueue header+payload as one TCP write so a retry never duplicates only
  // part of a WebSocket frame.
  e = tcp_write(conn->pcb, g_ws_tx_frame, frame_len, TCP_WRITE_FLAG_COPY);
  if (e != ERR_OK) return false;
  if (tcp_output(conn->pcb) != ERR_OK) return false;
  conn->last_activity_ms = sys_now();
  return true;
}

bool ws_conn_send_text(ws_conn_t *conn, const uint8_t *data, uint16_t len) {
  return ws_send(conn, 0x1, data, len);
}

bool ws_conn_send_binary(ws_conn_t *conn, const uint8_t *data, uint16_t len) {
  return ws_send(conn, 0x2, data, len);
}

void ws_conn_close(ws_conn_t *conn) {
  err_t e;
  if (!conn || !conn->pcb) return;

  tcp_arg(conn->pcb, NULL);
  tcp_recv(conn->pcb, NULL);
  tcp_sent(conn->pcb, NULL);
  tcp_poll(conn->pcb, NULL, 0);
  tcp_err(conn->pcb, NULL);

  e = tcp_close(conn->pcb);
  if (e != ERR_OK) tcp_abort(conn->pcb);
  conn->pcb = NULL;

  if (g_handler && g_handler->on_close) g_handler->on_close(conn);
  free(conn);
}

void ws_conn_close_with_reason(ws_conn_t *conn, uint16_t code, const char *reason) {
  uint8_t payload[125];
  size_t rlen = 0;
  uint16_t plen;

  if (!conn || !conn->pcb) return;

  if (conn->upgraded && code >= 1000u) {
    if (reason) rlen = strlen(reason);
    if (rlen > sizeof(payload) - 2u) rlen = sizeof(payload) - 2u;
    payload[0] = (uint8_t)(code >> 8);
    payload[1] = (uint8_t)(code & 0xFFu);
    if (rlen > 0) memcpy(payload + 2, reason, rlen);
    plen = (uint16_t)(2u + rlen);
    (void)ws_send(conn, 0x8, payload, plen);
  }

  ws_conn_close(conn);
}

static bool http_get_header(const char *req, const char *name, char *out, int out_sz) {
  char pattern[64];
  const char *p;
  const char *e;
  int n;

  snprintf(pattern, sizeof(pattern), "%s:", name);
  p = strstr(req, pattern);
  if (!p) return false;

  p += strlen(pattern);
  while (*p == ' ') p++;
  e = strstr(p, "\r\n");
  if (!e) return false;

  n = (int)(e - p);
  if (n <= 0 || n >= out_sz) return false;
  memcpy(out, p, (size_t)n);
  out[n] = 0;
  return true;
}

static void ws_make_accept(const char *client_key, char out_b64[64]) {
  const char *guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  char combo[128];
  sha1_ctx_t ctx;
  uint8_t digest[20];

  snprintf(combo, sizeof(combo), "%s%s", client_key, guid);
  sha1_init(&ctx);
  sha1_update(&ctx, (const uint8_t *)combo, strlen(combo));
  sha1_final(&ctx, digest);
  base64_encode(digest, 20, out_b64, 64);
}

static int ws_try_parse(const uint8_t *buf, int buflen, ws_frame_t *out) {
  uint8_t b0;
  uint8_t b1;
  bool fin;
  bool masked;
  uint8_t opcode;
  uint64_t len;
  int pos = 2;

  if (buflen < 2) return 0;

  b0 = buf[0];
  b1 = buf[1];
  fin = (b0 & 0x80) != 0;
  masked = (b1 & 0x80) != 0;
  opcode = b0 & 0x0F;
  len = (uint64_t)(b1 & 0x7F);

  if (!fin || !masked) return -1;

  if (len == 126) {
    if (buflen < pos + 2) return 0;
    len = ((uint16_t)buf[pos] << 8) | buf[pos + 1];
    pos += 2;
  } else if (len == 127) {
    return -1;
  }

  if (buflen < pos + 4) return 0;
  {
    uint8_t mask[4] = {buf[pos], buf[pos + 1], buf[pos + 2], buf[pos + 3]};
    pos += 4;

    if (len > sizeof(out->payload)) return -1;
    if (buflen < pos + (int)len) return 0;

    out->opcode = opcode;
    out->len = (uint16_t)len;
    for (uint16_t i = 0; i < out->len; i++) out->payload[i] = buf[pos + i] ^ mask[i & 3];
  }

  return pos + (int)len;
}

static void send_404_and_close(ws_conn_t *conn) {
  static const char nf[] = "HTTP/1.1 404 Not Found\r\nContent-Length:0\r\nConnection: close\r\n\r\n";
  if (conn && conn->pcb) {
    (void)tcp_write_all(conn->pcb, nf, (u16_t)strlen(nf));
    (void)tcp_output(conn->pcb);
  }
  ws_conn_close(conn);
}

static err_t on_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
  ws_conn_t *conn = (ws_conn_t *)arg;
  (void)err;

  if (!p) {
    ws_conn_close(conn);
    return ERR_OK;
  }

  if (!conn) {
    pbuf_free(p);
    return ERR_OK;
  }

  int copy = (int)p->tot_len;
  if (copy > (int)sizeof(conn->rx) - conn->rx_len) copy = (int)sizeof(conn->rx) - conn->rx_len;
  if (copy > 0) {
    pbuf_copy_partial(p, conn->rx + conn->rx_len, copy, 0);
    conn->rx_len += copy;
  }

  tcp_recved(tpcb, p->tot_len);
  conn->last_activity_ms = sys_now();
  pbuf_free(p);

  if (!conn->upgraded) {
    bool have_headers = false;
    for (int i = 0; i + 3 < conn->rx_len; i++) {
      if (conn->rx[i] == '\r' && conn->rx[i + 1] == '\n' && conn->rx[i + 2] == '\r' && conn->rx[i + 3] == '\n') {
        have_headers = true;
        break;
      }
    }

    if (!have_headers) {
      if (conn->rx_len >= (int)sizeof(conn->rx)) send_404_and_close(conn);
      return ERR_OK;
    }

    if (conn->rx_len >= (int)sizeof(conn->rx)) {
      send_404_and_close(conn);
      return ERR_OK;
    }

    conn->rx[conn->rx_len] = 0;

    if (strncmp((char *)conn->rx, "GET /ws ", 8) != 0) {
      send_404_and_close(conn);
      return ERR_OK;
    }

    {
      char key[128];
      char accept[64];
      char resp[256];
      int n;

      if (!http_get_header((char *)conn->rx, "Sec-WebSocket-Key", key, sizeof(key))) {
        send_404_and_close(conn);
        return ERR_OK;
      }

      ws_make_accept(key, accept);
      n = snprintf(resp, sizeof(resp),
                   "HTTP/1.1 101 Switching Protocols\r\n"
                   "Upgrade: websocket\r\n"
                   "Connection: Upgrade\r\n"
                   "Sec-WebSocket-Accept: %s\r\n"
                   "\r\n",
                   accept);

      if (tcp_write_all(tpcb, resp, (u16_t)n) != ERR_OK || tcp_output(tpcb) != ERR_OK) {
        ws_conn_close(conn);
        return ERR_OK;
      }

      conn->upgraded = true;
      conn->rx_len = 0;
      if (g_handler && g_handler->on_open) g_handler->on_open(conn);
      return ERR_OK;
    }
  }

  while (conn->rx_len > 0) {
    ws_frame_t fr;
    int used = ws_try_parse(conn->rx, conn->rx_len, &fr);
    if (used == 0) break;
    if (used < 0) {
      ws_conn_close(conn);
      return ERR_OK;
    }

    memmove(conn->rx, conn->rx + used, (size_t)(conn->rx_len - used));
    conn->rx_len -= used;

    if (fr.opcode == 0x8) {
      ws_send(conn, 0x8, NULL, 0);
      ws_conn_close(conn);
      return ERR_OK;
    }
    if (fr.opcode == 0x9) {
      ws_send(conn, 0xA, fr.payload, fr.len);
      continue;
    }
    if (fr.opcode == 0x1) {
      if (g_handler && g_handler->on_text) g_handler->on_text(conn, fr.payload, fr.len);
      continue;
    }
    if (fr.opcode == 0x2) {
      if (g_handler && g_handler->on_binary) g_handler->on_binary(conn, fr.payload, fr.len);
      continue;
    }
  }

  return ERR_OK;
}

static err_t on_poll(void *arg, struct tcp_pcb *tpcb) {
  ws_conn_t *conn = (ws_conn_t *)arg;
  uint32_t now_ms = sys_now();
  uint32_t idle_ms;

  if (!conn || !tpcb) return ERR_OK;

  if (conn->upgraded) return ERR_OK;

  idle_ms = now_ms - conn->last_activity_ms;
  if (idle_ms >= WS_HANDSHAKE_TIMEOUT_MS) {
    ws_conn_close(conn);
    return ERR_ABRT;
  }
  return ERR_OK;
}

static void on_err(void *arg, err_t err) {
  ws_conn_t *conn = (ws_conn_t *)arg;
  (void)err;
  if (!conn) return;
  conn->pcb = NULL;
  if (g_handler && g_handler->on_close) g_handler->on_close(conn);
  free(conn);
}

static err_t on_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
  ws_conn_t *conn;
  (void)arg;
  (void)err;

  conn = (ws_conn_t *)calloc(1, sizeof(ws_conn_t));
  if (!conn) {
    tcp_abort(newpcb);
    return ERR_ABRT;
  }

  conn->pcb = newpcb;
  conn->last_activity_ms = sys_now();
  tcp_arg(newpcb, conn);
  tcp_recv(newpcb, on_recv);
  tcp_poll(newpcb, on_poll, 10);
  tcp_err(newpcb, on_err);
  tcp_nagle_disable(newpcb);
  return ERR_OK;
}

bool web_server_start(const ws_app_handler_t *handler) {
  g_handler = handler;

  g_listen = tcp_new_ip_type(IPADDR_TYPE_ANY);
  if (!g_listen) return false;

  if (tcp_bind(g_listen, IP_ANY_TYPE, WS_SERVER_PORT) != ERR_OK) return false;
  g_listen = tcp_listen_with_backlog(g_listen, 4);
  if (!g_listen) return false;

  tcp_accept(g_listen, on_accept);
  return true;
}
