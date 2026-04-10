"""
Pre-build script: minifies src/config.html -> src/config.min.html
Registered via extra_scripts = pre:scripts/minify_html.py in platformio.ini.

The minifier:
  - removes HTML comments (<!-- ... -->)
  - strips leading / trailing whitespace from every line
  - drops blank lines
Runtime placeholders (SHORTVALS1, BATTERYENABLED, BATTERYSECTIONSTYLE, etc.)
are plain text / attribute values so they survive unchanged.
"""

import os
import re

import subprocess
import sys

def ensure_package(pkg):
    try:
        __import__(pkg)
    except ImportError:
        print(f"Installing missing Python package: {pkg}")
        subprocess.check_call([sys.executable, "-m", "pip", "install", pkg])


# Ensure dependency exists BEFORE using it
ensure_package("htmlmin")
ensure_package("csscompressor")
ensure_package("beautifulsoup4")
ensure_package("rjsmin")

from bs4 import BeautifulSoup
import htmlmin
import csscompressor
from rjsmin import jsmin

Import("env")  # noqa: F821  (PlatformIO injects this)


# def _minify(content: str) -> str:
#     # Remove HTML comments
#     content = re.sub(r"<!--.*?-->", "", content, flags=re.DOTALL)
#     # Strip leading/trailing whitespace per line and drop blank lines
#     lines = [line.strip() for line in content.splitlines()]
#     lines = [line for line in lines if line]
#     return "\n".join(lines)

# def _minify(content: str) -> str:
#     return htmlmin.minify(
#         content,
#         remove_comments=True,
#         remove_empty_space=True,
#         remove_all_empty_space=False,  # safer
#         reduce_boolean_attributes=True,
#         remove_optional_attribute_quotes=False
#     )
from bs4 import BeautifulSoup
import htmlmin
import csscompressor
from rjsmin import jsmin


def _minify(html: str) -> str:
    #return html # temporary because issues with minify

    soup = BeautifulSoup(html, "html.parser")

    # Minify CSS
    for style_tag in soup.find_all("style"):
        if style_tag.string:
            style_tag.string.replace_with(
                csscompressor.compress(style_tag.string)
            )

    # Minify JS
    for script_tag in soup.find_all("script"):
        # Skip external scripts
        if script_tag.get("src"):
            continue

        if script_tag.string:
            try:
                minified_js = jsmin(script_tag.string)
                script_tag.string.replace_with(minified_js)
            except Exception as e:
                print(f"JS minify failed: {e}")

    # Convert back to string
    html_str = str(soup)

    # Minify HTML
    # html_str = htmlmin.minify(
    #     html_str,
    #     remove_comments=True,
    #     remove_empty_space=True,
    #     reduce_boolean_attributes=True
    # )

    return html_str

def minify_html(*_args, **_kwargs):
    project_dir = env.subst("$PROJECT_DIR")  # noqa: F821
    src = os.path.join(project_dir, "src", "config.html")
    dst = os.path.join(project_dir, "src", "config.min.html")

    with open(src, "r", encoding="utf-8") as f:
        original = f.read()

    minified = _minify(original)

    with open(dst, "w", encoding="utf-8") as f:
        f.write(minified)

    print(
        f"minify_html: {os.path.getsize(src)} -> {os.path.getsize(dst)} bytes"
        f" ({100.0 * os.path.getsize(dst) / os.path.getsize(src):.1f}%)"
    )


# Run immediately when the pre: script is loaded (before compilation starts)
print("Minify script is running")
minify_html()
