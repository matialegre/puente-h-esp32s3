# -*- coding: utf-8 -*-
"""
banco_vivo.py - panel de banco partido en dos: ESP32-S3 del puente H (izquierda)
y osciloscopio OWON TAO3104A (derecha), con TODO lo que se manda y se recibe.

Cualquiera que quiera tocar el ESP o el osciloscopio (la pagina, Claude por curl,
un script) pasa por este servidor, asi queda una sola bitacora en pantalla:

    GET /                         la pagina
    GET /api/esp?c=<cmd>&src=x    manda un comando por serie al ESP (queda en el log)
    GET /api/scope?c=<scpi>&src=x manda un comando SCPI al osciloscopio (queda en el log)
    GET /api/log?desde=N          entradas del log a partir de N
    GET /api/wave                 forma de onda de pantalla de CH1..CH4 (no se loguea)
    GET /api/estado               JSON de estado del ESP (comando 'j', no se loguea)

Uso:  python banco_vivo.py [COM4]      -> http://127.0.0.1:8099/
"""
import json
import os
import socket
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import serial
from serial.tools import list_ports

AQUI = os.path.dirname(os.path.abspath(__file__))
PUERTO_WEB = 8099
OWON = ("169.254.10.100", 3000)
CUENTAS_POR_DIV = 6400        # dato de pantalla del OWON: 6400 cuentas = 1 division
UNID_OFFSET_POR_DIV = 50      # el OFFSET del encabezado viene en 1/50 de division

_log, _log_lock = [], threading.Lock()
_ser, _ser_lock = None, threading.Lock()
_scope_lock = threading.Lock()


def anotar(dev, src, cmd, resp):
    with _log_lock:
        _log.append({"n": len(_log), "t": time.strftime("%H:%M:%S"),
                     "dev": dev, "src": src, "cmd": cmd, "resp": resp})


# ---------------------------------------------------------------- ESP32
def puerto_esp():
    if len(sys.argv) > 1:
        return sys.argv[1]
    return next((p.device for p in list_ports.comports() if p.vid == 0x303A), None)


def esp(cmd):
    """Manda una linea y junta lo que conteste hasta que se calle 150 ms."""
    global _ser
    with _ser_lock:
        try:
            if _ser is None:
                com = puerto_esp()
                if not com:
                    return "(no encuentro el ESP32-S3 en ningun COM)"
                s = serial.Serial()
                s.port, s.baudrate, s.timeout, s.write_timeout = com, 115200, 0.05, 3
                s.dsrdtr = s.rtscts = False
                s.open()
                s.dtr, s.rts = True, False   # el USB nativo del S3 no habla sin DTR
                time.sleep(0.3)
                _ser = s
            _ser.reset_input_buffer()
            _ser.write((cmd + "\n").encode())
            _ser.flush()
            out, quieto = b"", time.time()
            while time.time() - quieto < 0.15 or not out:
                d = _ser.read(4096)
                if d:
                    out += d
                    quieto = time.time()
                elif time.time() - quieto > 1.5:
                    break
            return out.decode("utf-8", "replace").strip()
        except Exception as e:
            try:
                _ser.close()
            except Exception:
                pass
            _ser = None
            return "(error serie: %s)" % e


# ---------------------------------------------------------------- OWON
def scope_raw(cmd, timeout=1.5):
    with _scope_lock, socket.create_connection(OWON, timeout=3) as s:
        s.settimeout(timeout)
        s.sendall((cmd + "\n").encode())
        if not cmd.rstrip().endswith("?"):
            time.sleep(0.15)
            return b""
        out = b""
        try:
            while True:
                d = s.recv(65536)
                if not d:
                    break
                out += d
                if out.endswith(b"->") or out.endswith(b"\n"):
                    break
                # respuestas binarias: 4 bytes de largo + datos
                if len(out) >= 4 and cmd.upper().startswith(":DATA"):
                    if len(out) >= 4 + struct.unpack("<I", out[:4])[0]:
                        break
        except socket.timeout:
            pass
        return out


def scope(cmd):
    try:
        r = scope_raw(cmd)
        return r.decode(errors="replace").strip().removesuffix("->").strip() if r else "(ok, sin respuesta)"
    except Exception as e:
        return "(error red: %s)" % e


def wave():
    # Congela la pantalla antes de leer: cada canal se pide con un comando
    # aparte, y con el osciloscopio adquiriendo CH1 y CH2 podian salir de
    # barridos distintos, con una fase cualquiera entre ellos (paso el
    # 4-oct: el bipolar "fuera de fase" era eso).
    scope_raw(":RUNning STOP")
    time.sleep(0.25)
    try:
        return _wave_congelada()
    finally:
        # Probado en este OWON (fw V2.1.0): despues de STOP, "AUTO" solo no
        # vuelve a adquirir; hace falta "RUN" y despues "AUTO".
        scope_raw(":RUNning RUN")
        scope_raw(":RUNning AUTO")


def _wave_congelada():
    h = scope_raw(":DATA:WAVE:SCREen:HEAD?")
    head = json.loads(h[4:].decode(errors="replace"))
    canales = []
    for c in head["CHANNEL"]:
        if c["DISPLAY"] != "ON":
            continue
        r = scope_raw(":DATA:WAVE:SCREEN:%s?" % c["NAME"])
        n = struct.unpack("<I", r[:4])[0]
        crudo = struct.unpack("<%dh" % (n // 2), r[4:4 + n])
        e = c["SCALE"]                       # "5.00V" o "500mV"
        escala = float(e[:-2]) / 1000 if e.endswith("mV") else float(e[:-1])
        off_div = c["OFFSET"] / UNID_OFFSET_POR_DIV
        divs = [v / CUENTAS_POR_DIV for v in crudo]
        volts = [(d - off_div) * escala for d in divs]
        canales.append({"nombre": c["NAME"], "escala": c["SCALE"], "probe": c["PROBE"],
                        "off_div": off_div, "divs": divs[::2], "volts": volts})
    return {"base": head["TIMEBASE"]["SCALE"], "muestras": head["SAMPLE"]["DATALEN"],
            "canales": canales}


# ---------------------------------------------------------------- HTTP
class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj):
        b = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        src = q.get("src", "pagina")
        if u.path in ("/", "/index.html"):
            b = open(os.path.join(AQUI, "banco_vivo.html"), "rb").read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(b)
        elif u.path == "/api/esp":
            r = esp(q.get("c", ""))
            anotar("esp", src, q.get("c", ""), r)
            self._json({"resp": r})
        elif u.path == "/api/scope":
            r = scope(q.get("c", ""))
            anotar("scope", src, q.get("c", ""), r)
            self._json({"resp": r})
        elif u.path == "/api/log":
            d = int(q.get("desde", 0))
            with _log_lock:
                self._json(_log[d:])
        elif u.path == "/api/wave":
            try:
                self._json(wave())
            except Exception as e:
                self._json({"error": str(e)})
        elif u.path == "/api/captura":
            # guarda la pantalla del osciloscopio + el estado del ESP, para el informe
            try:
                nombre = "".join(ch for ch in q.get("nombre", "captura") if ch.isalnum() or ch in "-_")
                carpeta = os.path.join(AQUI, "informe_tp", "capturas")
                os.makedirs(carpeta, exist_ok=True)
                r = esp("j")
                datos = {"nombre": nombre, "hora": time.strftime("%Y-%m-%d %H:%M:%S"),
                         "esp": json.loads(r[r.index("{"):r.rindex("}") + 1]), "osc": wave()}
                ruta = os.path.join(carpeta, nombre + ".json")
                json.dump(datos, open(ruta, "w"))
                anotar("scope", src, "CAPTURA " + nombre, "guardada en " + ruta)
                self._json({"ok": ruta})
            except Exception as e:
                self._json({"error": str(e)})
        elif u.path == "/api/estado":
            r = esp("j")
            try:
                self._json(json.loads(r[r.index("{"):r.rindex("}") + 1]))
            except Exception:
                self._json({"error": r})
        else:
            self.send_error(404)


if __name__ == "__main__":
    print("ESP en:", puerto_esp(), "| osciloscopio:", OWON[0])
    print("ABRIR: http://127.0.0.1:%d/" % PUERTO_WEB)
    ThreadingHTTPServer(("127.0.0.1", PUERTO_WEB), H).serve_forever()
