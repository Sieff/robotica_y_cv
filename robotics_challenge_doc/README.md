## Setup

1. Öffne Projekt in VSCode
2. Installiere Latex Workshop und LTex
3. Öffne settings.json (ctrl + shift + p -> Open user settings (json))
4. Paste folgendes Snippet:

```
    "latex-workshop.latex.outputDir": "./out/",
    "latex-workshop.latex.outDir": "out",
    "latex-workshop.latex.tools": [
        {
         "name": "latexmk",
         "command": "latexmk",
         "args": [
          "-synctex=1",
          "-interaction=nonstopmode",
          "-file-line-error",
          "-pdf",
          "-outdir=out",
          "-output-directory=out",
          "%DOC%"
         ],
         "env": {}
        },
        {
         "name": "xelatex",
         "command": "xelatex",
         "args": [
          "-synctex=1",
          "-interaction=nonstopmode",
          "-file-line-error",
          "-aux-directory=out",
          "-output-directory=out",
          "%DOC%"
         ],
         "env": {}
        },
        {
         "name": "pdflatex",
         "command": "pdflatex",
         "args": [
          "-synctex=1",
          "-interaction=nonstopmode",
          "-file-line-error",
          "-aux-directory=out",
          "-output-directory=out",
          "%DOC%"
         ],
         "env": {}
        },
        {
         "name": "bibtex",
         "command": "bibtex",
         "args": [
          "%OUTDIR%/%DOCFILE%"
         ],
         "env": {}
        },
        {
            "name": "biber",
            "command": "biber",
            "args": [
                "%OUTDIR%/%DOCFILE%"
            ]
        },
    ],
    "latex-workshop.latex.recipes": [
        {
            "name": "pdflatex ➞ biber ➞ pdflatex`×2",
            "tools": [
             "pdflatex",
             "biber",
             "pdflatex",
             "pdflatex"
            ]
        },
        {
            "name": "pdflatex ➞ bibtex ➞ pdflatex`×2",
            "tools": [
             "pdflatex",
             "bibtex",
             "pdflatex",
             "pdflatex"
            ]
        },
        {
         "name": "pdfLaTeX",
         "tools": [
          "pdflatex"
         ]
        },
        {
         "name": "biber",
         "tools": [
          "biber"
         ]
        },
        {
         "name": "bibtex",
         "tools": [
          "bibtex"
         ]
        },
        {
        "name": "xelatex ➞ biber ➞ xelatex`×2",
        "tools": [
          "xelatex",
          "biber",
          "xelatex",
          "xelatex"
         ]
        }
    ],
    "latex-workshop.latex.recipe.default": "pdfLaTeX",
```

5. Bauen mit Biber (ctrl + shift + p -> Latex Workshop: Build with Recipe)
6. Normales bauen über speichern

## Fix bib file

    biber --tool --configfile=remabs.conf --output-file=expose.bib --logfile=out/expose.bib.blg expose.bib