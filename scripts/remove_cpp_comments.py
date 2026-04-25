from __future__ import annotations

import argparse
import codecs
import subprocess
import sys
from pathlib import Path


RAW_PREFIXES = ("u8R\"", "LR\"", "uR\"", "UR\"", "R\"")


def find_raw_string_end(text: str, start: int) -> int | None:
    for prefix in RAW_PREFIXES:
        if not text.startswith(prefix, start):
            continue
        marker_start = start + len(prefix)
        open_paren = text.find("(", marker_start)
        if open_paren < 0:
            return None
        delimiter = text[marker_start:open_paren]
        if any(ch.isspace() or ch in "\\()" for ch in delimiter):
            return None
        close_marker = ")" + delimiter + "\""
        close_pos = text.find(close_marker, open_paren + 1)
        if close_pos < 0:
            return len(text)
        return close_pos + len(close_marker)
    return None


def remove_cpp_comments(text: str) -> str:
    out: list[str] = []
    i = 0
    n = len(text)

    while i < n:
        raw_end = find_raw_string_end(text, i)
        if raw_end is not None:
            out.append(text[i:raw_end])
            i = raw_end
            continue

        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""

        if c == "/" and nxt == "/":
            i += 2
            while i < n and text[i] not in "\r\n":
                i += 1
            continue

        if c == "/" and nxt == "*":
            if out and not out[-1].endswith((" ", "\t", "\r", "\n")):
                out.append(" ")
            i += 2
            while i < n:
                if i + 1 < n and text[i] == "*" and text[i + 1] == "/":
                    i += 2
                    break
                if text[i] in "\r\n":
                    out.append(text[i])
                    if text[i] == "\r" and i + 1 < n and text[i + 1] == "\n":
                        i += 1
                        out.append(text[i])
                i += 1
            continue

        if c == "\"":
            quote = c
            out.append(c)
            i += 1
            while i < n:
                out.append(text[i])
                if text[i] == "\\":
                    i += 1
                    if i < n:
                        out.append(text[i])
                elif text[i] == quote:
                    i += 1
                    break
                i += 1
            continue

        if c == "'" and not is_digit_separator(text, i):
            out.append(c)
            i += 1
            while i < n:
                out.append(text[i])
                if text[i] == "\\":
                    i += 1
                    if i < n:
                        out.append(text[i])
                elif text[i] == "'":
                    i += 1
                    break
                i += 1
            continue

        out.append(c)
        i += 1

    return "".join(out)


def is_digit_separator(text: str, index: int) -> bool:
    prev = text[index - 1] if index > 0 else ""
    nxt = text[index + 1] if index + 1 < len(text) else ""
    return (prev.isalnum() or prev == "_") and (nxt.isalnum() or nxt == "_")


def strip_trailing_whitespace(text: str) -> str:
    normalized = text.replace("\r\n", "\n").replace("\r", "\n")
    lines = normalized.splitlines(keepends=True)
    cleaned: list[str] = []
    for line in lines:
        ending = ""
        body = line
        if line.endswith("\n"):
            body = line[:-1]
            ending = "\n"
        cleaned.append(body.rstrip(" \t") + ending)
    return "".join(cleaned)


def clean_cpp_source(text: str) -> str:
    return strip_trailing_whitespace(remove_cpp_comments(text))


def tracked_source_files(root: Path) -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "--", "*.h", "*.cpp"],
        cwd=root,
        check=True,
        text=True,
        capture_output=True,
    )
    return [root / line.strip() for line in result.stdout.splitlines() if line.strip()]


def read_text(path: Path) -> tuple[str, bool]:
    data = path.read_bytes()
    has_bom = data.startswith(codecs.BOM_UTF8)
    return data.decode("utf-8-sig" if has_bom else "utf-8"), has_bom


def write_text(path: Path, text: str, has_bom: bool) -> None:
    encoded = text.encode("utf-8")
    if has_bom:
        encoded = codecs.BOM_UTF8 + encoded
    path.write_bytes(encoded)


def run_self_test() -> None:
    cases = [
        ("int a; // drop\n", "int a;\n"),
        ("int a/* drop */b;\n", "int a b;\n"),
        ("auto s = \"// keep /* keep */\"; // drop\n", "auto s = \"// keep /* keep */\";\n"),
        ("auto c = '/'; /* drop\nmore */\nint x;\n", "auto c = '/';\n\nint x;\n"),
        ("auto x = 0x7FFF'FFFF; // drop\n", "auto x = 0x7FFF'FFFF;\n"),
        ("auto r = R\"tag(// keep\n/* keep */)tag\"; // drop\n", "auto r = R\"tag(// keep\n/* keep */)tag\";\n"),
        ("auto r = u8R\"(http://keep/*too*/)\"; /* drop */\n", "auto r = u8R\"(http://keep/*too*/)\";\n"),
    ]
    for idx, (source, expected) in enumerate(cases, 1):
        actual = clean_cpp_source(source)
        if actual != expected:
            raise AssertionError(f"case {idx} failed:\nactual:   {actual!r}\nexpected: {expected!r}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("paths", nargs="*")
    parser.add_argument("--root", default=".")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args(argv)

    if args.self_test:
        run_self_test()
        return 0

    root = Path(args.root).resolve()
    files = [Path(p).resolve() for p in args.paths] if args.paths else tracked_source_files(root)
    changed: list[Path] = []

    for path in files:
        if path.suffix.lower() not in {".h", ".cpp"}:
            continue
        original, has_bom = read_text(path)
        updated = clean_cpp_source(original)
        if updated != original:
            changed.append(path)
            if not args.check:
                write_text(path, updated, has_bom)

    for path in changed:
        print(path.relative_to(root))

    if args.check and changed:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
