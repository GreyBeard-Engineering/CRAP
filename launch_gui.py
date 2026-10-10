"""Start the gearbox GUI server and open it in the default browser.

Run:  python launch_gui.py [--port 8000] [--no-browser]
"""
import argparse
import threading
import time
import urllib.request
import webbrowser

import uvicorn


def _open_when_ready(url: str, timeout: float = 120.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            urllib.request.urlopen(url + "api/defaults", timeout=1)
            webbrowser.open(url)
            return
        except OSError:
            time.sleep(0.3)
    print(f"server did not come up within {timeout:.0f} s; open {url} manually")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    url = f"http://127.0.0.1:{args.port}/"   # not "localhost": it can resolve to IPv6 first and stall
    print(f"gearbox GUI at {url}  (Ctrl+C to stop)")
    if not args.no_browser:
        threading.Thread(target=_open_when_ready, args=(url,), daemon=True).start()
    uvicorn.run("web.server:app", host="127.0.0.1", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
