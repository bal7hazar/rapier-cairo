import sys,re,pathlib
# Truncate each file at the trailing `#[cfg(test)]\nmod ... {` block (upstream unit tests), if it is the last top-level item.
for p in pathlib.Path(sys.argv[1]).rglob('*.cairo'):
    s=p.read_text()
    m=re.search(r'\n(// Tests[^\n]*\n[^\n]*\n)?\s*#\[cfg\(test\)\]\nmod \w+ \{', s)
    if m:
        rest=s[m.start():]
        # verify block runs to EOF: last line is a lone "}"
        assert rest.rstrip().endswith('}')
        # ensure no top-level item after it: find closing "\n}\n" first occurrence at col 0
        idx=rest.find('\n}\n')
        tail=rest[idx+3:].strip()
        if tail:
            print("NOT AT END", p, repr(tail[:80])); continue
        p.write_text(s[:m.start()]+'\n')
        print("stripped", p)
