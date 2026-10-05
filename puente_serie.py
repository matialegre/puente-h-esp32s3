# Puente HTTP <-> puerto serie para el panel del puente H (ESP32-S3).
#
# El navegador no puede abrir un COM. Este servidor expone en :8088 la misma
# API que el ESP32 sirve por WiFi, y la traduce a comandos del puerto serie:
#
#   GET /            -> sirve motor_ui.html
#   GET /estado      -> manda 'j' y devuelve el JSON tal cual lo arma el firmware
#   GET /cmd?m=..&v=..&d=..&f=..  -> aplica y devuelve el estado
#
# Asi el panel anda con el USB enchufado, sin tener que pasarse a la red
# PuenteH y quedarse sin internet en la PC.
#
# El estado NO se parsea de texto: el firmware tiene un comando 'j' que
# escupe el mismo JSON que sirve por HTTP. Una sola fuente para el estado;
# si manana se agrega un campo, aparece de los dos lados sin tocar esto.
#
# Uso:  python puente_serie.py [COM16] [8088]

import http.server, os, sys, threading, time, json
import serial

PUERTO_COM = sys.argv[1] if len(sys.argv) > 1 else 'COM16'
PUERTO_WEB = int(sys.argv[2]) if len(sys.argv) > 2 else 8088
HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'motor_ui.html')

_lock = threading.Lock()
_ser = None


def abrir():
    """Abre el COM. El USB nativo del S3 no acepta datos hasta que el host
    asierta DTR: sin eso el write se cuelga con 'Write timeout'."""
    global _ser
    s = serial.Serial()
    s.port = PUERTO_COM
    s.baudrate = 115200
    s.timeout = 0.4
    s.write_timeout = 3
    s.dsrdtr = False
    s.rtscts = False
    s.open()
    s.dtr = True
    s.rts = False
    time.sleep(2.0)          # el ESP32 se reinicia al abrir el puerto
    s.reset_input_buffer()
    _ser = s
    print('[serie] abierto %s' % PUERTO_COM)


def mandar(linea, espera=0.35):
    """Manda una linea y devuelve lo que conteste. Reabre el puerto si el
    ESP32 se reinicio o alguien lo desenchufo: es una herramienta de banco,
    tiene que sobrevivir a que muevan un cable."""
    global _ser
    with _lock:
        for intento in (1, 2):
            try:
                if _ser is None:
                    abrir()
                _ser.reset_input_buffer()
                _ser.write((linea + '\n').encode())
                _ser.flush()
                time.sleep(espera)
                n = _ser.in_waiting
                return _ser.read(n if n else 1).decode('utf-8', 'replace')
            except Exception as e:
                print('[serie] %s -> reintento (%s)' % (e, intento))
                try:
                    _ser.close()
                except Exception:
                    pass
                _ser = None
                if intento == 2:
                    raise
    return ''


def estado():
    """Pide el JSON al firmware. Devuelve None si no vino nada usable, para
    que el panel muestre 'sin conexion' en vez de un estado inventado."""
    txt = mandar('j', 0.45)
    for linea in txt.splitlines():
        linea = linea.strip()
        if linea.startswith('{') and linea.endswith('}'):
            try:
                return json.loads(linea)
            except ValueError:
                pass
    return None


class H(http.server.BaseHTTPRequestHandler):

    def log_message(self, *a):
        pass

    def _responder(self, cod, tipo, cuerpo):
        if isinstance(cuerpo, str):
            cuerpo = cuerpo.encode('utf-8')
        self.send_response(cod)
        self.send_header('Content-Type', tipo)
        self.send_header('Content-Length', str(len(cuerpo)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(cuerpo)

    def do_GET(self):
        from urllib.parse import urlparse, parse_qs
        u = urlparse(self.path)
        q = parse_qs(u.query)

        if u.path in ('/', '/motor_ui.html'):
            try:
                with open(HTML, 'rb') as f:
                    return self._responder(200, 'text/html; charset=utf-8', f.read())
            except Exception as e:
                return self._responder(500, 'text/plain; charset=utf-8',
                                       'No se pudo leer motor_ui.html: %s' % e)

        if u.path not in ('/estado', '/cmd'):
            return self._responder(404, 'text/plain', 'no existe')

        try:
            if u.path == '/cmd':
                # El orden importa: primero frecuencia y tiempo muerto, que
                # reconfiguran el timer, y al final el modo y la velocidad,
                # que son lo que el usuario acaba de tocar.
                if 'f' in q: mandar('h %d' % int(float(q['f'][0])))
                if 'd' in q: mandar('d %d' % int(float(q['d'][0])))
                if 'm' in q:
                    m = q['m'][0][:1]
                    if m in 'arfsub':
                        mandar(m)
                if 'v' in q: mandar('v %s' % float(q['v'][0]))
            e = estado()
            if e is None:
                return self._responder(502, 'application/json',
                                       '{"error":"el ESP32 no contesto por el puerto serie"}')
            return self._responder(200, 'application/json', json.dumps(e))
        except Exception as e:
            return self._responder(502, 'application/json',
                                   json.dumps({'error': str(e)}))


if __name__ == '__main__':
    try:
        abrir()
    except Exception as e:
        print('No pude abrir %s: %s' % (PUERTO_COM, e))
        print('Cerra el monitor serie (VS Code, IDE de Arduino, PuTTY) y volve a probar.')
        raise SystemExit(1)

    e = estado()
    print('[serie] el ESP32 contesta:', e if e else 'NADA — revisa el firmware')

    srv = http.server.ThreadingHTTPServer(('127.0.0.1', PUERTO_WEB), H)
    print('\n  Abri  http://127.0.0.1:%d\n' % PUERTO_WEB)
    print('  En la casilla de arriba a la derecha del panel pone:')
    print('      http://127.0.0.1:%d\n' % PUERTO_WEB)
    print('  Ctrl+C para cortar.')
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\ncortado')
