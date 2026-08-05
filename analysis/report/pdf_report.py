from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import shutil
import subprocess


@dataclass
class ReportSection:
    title: str
    content: str


class PdfReport:
    """
    Simple builder for LaTeX/PDF reports.

    Example
    -------
    report = PdfReport(output_dir)

    report.add_section(
        "Absolute Metrics",
        absolute_table_latex
    )

    report.add_section(
        "Comparative Metrics",
        comparison_table_latex
    )

    pdf_path = report.build()
    """

    def __init__(
        self,
        output_dir: Path,
        export_dirs=None,
        title: str = "Results",
        keep_tex: bool = True,
    ):
        self.output_dir = Path(output_dir)

        self.export_dirs = [
          Path(d)
          for d in (export_dirs or [])
        ]

        self.output_dir.mkdir(
          parents=True,
          exist_ok=True
        )

        for d in self.export_dirs:
          d.mkdir(
            parents=True,
            exist_ok=True
          )

        self.title = title
        self.keep_tex = keep_tex

        self.sections: list[ReportSection] = []

    # --------------------------------------------------------
    # Public API
    # --------------------------------------------------------

    def add_section(
        self,
        title: str,
        content: Optional[str],
    ) -> None:
        """
        Add a section to the report.

        Empty sections are ignored.
        """

        if not content:
            return

        self.sections.append(
            ReportSection(title, content)
        )

    def build(
        self,
        filename: str = "results_report",
    ) -> Path:
        """
        Generates .tex, compiles PDF,
        and returns the final PDF path.
        """

        tex_file = self.output_dir / f"{filename}.tex"

        tex_file.write_text(
            self._generate_document(),
            encoding="utf-8",
        )

        pdf_file = self._compile(tex_file)

        # -------------------------------------------------
        # Export copies
        # -------------------------------------------------

        self._export_artifact(pdf_file)

        if self.keep_tex:
          self._export_artifact(tex_file)

        if not self.keep_tex:
            tex_file.unlink(missing_ok=True)

        return pdf_file

    # --------------------------------------------------------
    # Internal
    # --------------------------------------------------------

    def _generate_document(self) -> str:

        body = []

        for section in self.sections:

            body.append(
                rf"""
\subsection*{{{section.title}}}

{section.content}
"""
            )

        return rf"""
\documentclass{{article}}

\usepackage{{booktabs}}
\usepackage{{longtable}}
\usepackage{{caption}}
\usepackage{{geometry}}
\geometry{{margin=1in}}

\begin{{document}}

\section*{{{self.title}}}

{''.join(body)}

\end{{document}}
"""

    def _compile(
        self,
        tex_file: Path,
    ) -> Path:

        build_dir = self.output_dir / "latex_build"

        build_dir.mkdir(
            parents=True,
            exist_ok=True,
        )

        subprocess.run(
            [
                "pdflatex",
                "-interaction=nonstopmode",
                "-halt-on-error",
                "-output-directory",
                str(build_dir),
                str(tex_file),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        pdf = build_dir / tex_file.with_suffix(".pdf").name

        final_pdf = self.output_dir / pdf.name

        shutil.move(pdf, final_pdf)

        shutil.rmtree(build_dir)

        return final_pdf

    def _export_artifact(self, file: Path):

      for directory in self.export_dirs:
        destination = directory / file.name

        shutil.copy2(
          file,
          destination
        )
