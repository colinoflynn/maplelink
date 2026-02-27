#include <string.h>

#include "lwip/apps/fs.h"

static const char INDEX_HTML[] =
  "<!doctype html><html><head><meta charset='utf-8'><title>Pico BrowserIO</title>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<style>"
  "body{margin:0;background:#0b0f14;color:#e6edf3;font:14px/1.4 monospace;}"
  ".bar{padding:10px;border-bottom:1px solid #1f2937;background:#111827;display:flex;gap:10px;align-items:center;flex-wrap:wrap}"
  "label{display:flex;gap:6px;align-items:center}"
  "input,select,button{background:#0f172a;color:#e6edf3;border:1px solid #334155;padding:4px 8px}"
  "#term{white-space:pre-wrap;word-break:break-word;padding:12px;height:calc(100vh - 56px);overflow-y:auto;outline:none}"
  ".ok{color:#22c55e}.err{color:#ef4444}"
  "</style>"
  "</head><body>"
  "<div class='bar'>"
  "<label>Baud <input id='baud' type='number' value='115200' min='300' step='100'></label>"
  "<label>Data <select id='data'><option>8</option><option>7</option></select></label>"
  "<label>Parity <select id='parity'><option value='none'>none</option><option value='even'>even</option><option value='odd'>odd</option></select></label>"
  "<label>Stop <select id='stop'><option value='1'>1</option><option value='2'>2</option></select></label>"
  "<button id='apply'>Apply</button>"
  "<span id='st'>connecting...</span>"
  "</div>"
  "<div id='term' tabindex='0'></div>"
  "<script>"
  "const term=document.getElementById('term');"
  "const st=document.getElementById('st');"
  "const ws=new WebSocket('ws://'+location.hostname+':81/ws');"
  "ws.binaryType='arraybuffer';"
  "const enc=new TextEncoder();"
  "const dec=new TextDecoder();"
  "function log(s,cls){const span=document.createElement('span');if(cls)span.className=cls;span.textContent=s;term.appendChild(span);term.scrollTop=term.scrollHeight;}"
  "function send(o){if(ws.readyState===1)ws.send(JSON.stringify(o));}"
  "ws.onopen=()=>{st.textContent='connected';st.className='ok';send({type:'hello'});send({type:'uart.get_config',port:'uart0'});term.focus();};"
  "ws.onclose=()=>{st.textContent='disconnected';st.className='err';};"
  "ws.onmessage=(e)=>{"
  " if(typeof e.data==='string'){"
  "  try{const m=JSON.parse(e.data);"
  "   if(m.type==='uart.config'){log('\\n[config '+JSON.stringify(m)+']\\n','ok');}"
  "   else if(m.type==='error'){log('\\n[error '+(m.msg||'unknown')+']\\n','err');}"
  "   else{log('\\n['+e.data+']\\n');}"
  "  }catch(_){log(e.data);}"
  " }else{log(dec.decode(new Uint8Array(e.data)));}"
  "};"
  "document.getElementById('apply').onclick=()=>{"
  " send({type:'uart.set_config',port:'uart0',baud:parseInt(document.getElementById('baud').value,10)||115200,"
  " data_bits:parseInt(document.getElementById('data').value,10)||8,parity:document.getElementById('parity').value,"
  " stop_bits:parseInt(document.getElementById('stop').value,10)||1,flow:'none'});"
  "};"
  "term.addEventListener('keydown',(e)=>{"
  " if(ws.readyState!==1)return;"
  " let out=null;"
  " if(e.key==='Enter')out=new Uint8Array([13]);"
  " else if(e.key==='Backspace')out=new Uint8Array([127]);"
  " else if(e.key.length===1)out=enc.encode(e.key);"
  " else return;"
  " ws.send(out);"
  " e.preventDefault();"
  "});"
  "</script></body></html>";

int fs_open_custom(struct fs_file *file, const char *name) {
  if (!file || !name) return 0;

  if (strcmp(name, "/") == 0 || strcmp(name, "/index.html") == 0) {
    file->data = INDEX_HTML;
    file->len = (int)strlen(INDEX_HTML);
    file->index = file->len;
    file->flags = 0;
#if LWIP_HTTPD_FILE_EXTENSION
    file->pextension = NULL;
#endif
#if HTTPD_PRECALCULATED_CHECKSUM
    file->chksum = NULL;
    file->chksum_count = 0;
#endif
#if LWIP_HTTPD_FILE_STATE
    file->state = NULL;
#endif
    return 1;
  }

  return 0;
}

void fs_close_custom(struct fs_file *file) {
  LWIP_UNUSED_ARG(file);
}
