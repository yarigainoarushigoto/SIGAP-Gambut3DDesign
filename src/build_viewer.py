#!/usr/bin/env python3
"""Menyisipkan data model (model/viewer_data.json) ke src/viewer_template.html -> index.html."""
import os
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
data = open(os.path.join(root, "model", "viewer_data.json")).read().replace("</", "<\\/")
tpl = open(os.path.join(root, "src", "viewer_template.html")).read()
html = "<!doctype html>\n<html lang=\"id\">\n<head>\n<meta charset=\"utf-8\">\n<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, viewport-fit=cover\">\n<style>html,body{margin:0}[hidden]{display:none!important}*,*::before,*::after{box-sizing:border-box}</style>\n" \
       + tpl.replace("__MODEL__", data, 1) + "\n</html>\n"
open(os.path.join(root, "index.html"), "w").write(html)
print("index.html dibuat")
