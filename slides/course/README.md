# MR2 Code Walkthrough Slides

Source files:

- `syllabus.md` - five-session meeting syllabus
- `lecture-01.md` - Marp deck for session 1
- `theme.css` - custom Marp theme
- `.marprc.yml` - Marp config

## Shell

```sh
cd ~/Developer/mr2-stack/slides
nix develop
```

## Export

Regenerate Mermaid diagrams:

```sh
cd ..
PUPPETEER_EXECUTABLE_PATH="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" \
  nix develop . -c sh -lc 'for f in course/diagrams/*.mmd; do b=$(basename "$f" .mmd); mmdc -i "$f" -o "course/assets/$b.svg" -b transparent; done'
```

HTML:

```sh
cd course
marp lecture-01.md --config .marprc.yml --html -o lecture-01.html
```

PDF:

```sh
cd course
marp lecture-01.md --config .marprc.yml --pdf -o lecture-01.pdf
```
