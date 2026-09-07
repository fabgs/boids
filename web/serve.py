#!/usr/bin/env python3
"""Servidor estatico para probar la version web en local.

Sirve la carpeta dist/ (salida de build_web.*) anadiendo las cabeceras COOP/COEP que exige el navegador
para habilitar SharedArrayBuffer, es decir, los hilos de la simulacion. Un `python -m http.server` normal
no las envia y la pagina fallaria al arrancar.

Uso: python web/serve.py [puerto]   (por defecto 8000) y abrir http://localhost:8000
"""
import functools
import http.server
import os
import sys


class Handler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **http.server.SimpleHTTPRequestHandler.extensions_map,
        ".wasm": "application/wasm",
        ".js": "text/javascript",
        ".data": "application/octet-stream",
    }

    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "dist")
    if not os.path.isfile(os.path.join(root, "index.html")):
        sys.exit("No existe dist/index.html: compila primero con build_web.bat o build_web.sh")
    server = http.server.ThreadingHTTPServer(("127.0.0.1", port), functools.partial(Handler, directory=root))
    print(f"Sirviendo {os.path.abspath(root)} en http://localhost:{port} (Ctrl+C para parar)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
