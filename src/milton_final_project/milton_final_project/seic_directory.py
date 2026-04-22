from dataclasses import dataclass
from difflib import SequenceMatcher
from pathlib import Path
import re
from typing import List, Optional
import zipfile
from xml.etree import ElementTree as ET

from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class DirectoryEntry:
    kind: str
    title: str
    location: str
    floor: str
    description: str
    notes: str


@dataclass(frozen=True)
class DirectoryMatch:
    query: str
    entry: Optional[DirectoryEntry]
    score: float


class SeicDirectory:
    def __init__(self, workbook_path: Optional[str] = None):
        if workbook_path is None:
            workbook_path = self._default_workbook_path()

        self.workbook_path = workbook_path
        self.entries = self._load_entries()

    def find_best_match(self, user_query: str) -> DirectoryMatch:
        cleaned_query = self._normalize_query(user_query)
        if not cleaned_query:
            return DirectoryMatch(query='', entry=None, score=0.0)

        best_entry = None
        best_score = 0.0

        for entry in self.entries:
            score = self._score_entry(cleaned_query, entry)
            if score > best_score:
                best_score = score
                best_entry = entry

        if best_score < 0.52:
            return DirectoryMatch(query=cleaned_query, entry=None, score=best_score)

        return DirectoryMatch(query=cleaned_query, entry=best_entry, score=best_score)

    def build_response(self, match: DirectoryMatch) -> str:
        if match.entry is None:
            return f"Sorry, I couldn't find {match.query} in the public SEIC directory."

        entry = match.entry
        if entry.kind == 'room':
            detail = f'{entry.title} is on floor {entry.floor}'
            if entry.description:
                detail += f' and is a {entry.description}'
            return detail + '.'

        if entry.kind == 'person':
            detail = f'{entry.title} is listed at {entry.location}'
            if entry.floor:
                detail += f' on floor {entry.floor}'
            return detail + '.'

        detail = f'{entry.title} is listed at {entry.location}'
        if entry.floor and entry.floor != '0':
            detail += f' on floor {entry.floor}'
        if 'not publicly listed' in entry.location.lower() or 'room not publicly listed' in entry.location.lower():
            detail += ', but a public room number is not listed'
        return detail + '.'

    def expression_for_match(self, match: DirectoryMatch) -> str:
        if match.entry is None:
            return 'confused'
        if match.entry.kind == 'person':
            return 'happy'
        return 'ready_to_go'

    def _load_entries(self) -> List[DirectoryEntry]:
        shared_strings = []
        namespace = {'a': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}

        with zipfile.ZipFile(self.workbook_path) as workbook:
            if 'xl/sharedStrings.xml' in workbook.namelist():
                root = ET.fromstring(workbook.read('xl/sharedStrings.xml'))
                for item in root.findall('a:si', namespace):
                    text = ''.join(node.text or '' for node in item.findall('.//a:t', namespace))
                    shared_strings.append(text)

            room_rows = self._sheet_rows(workbook, 'sheet2.xml', shared_strings, namespace)
            people_rows = self._sheet_rows(workbook, 'sheet3.xml', shared_strings, namespace)
            named_rows = self._sheet_rows(workbook, 'sheet4.xml', shared_strings, namespace)

        entries = []

        for row in room_rows[1:]:
            if len(row) < 8 or not row[0]:
                continue
            entries.append(
                DirectoryEntry(
                    kind='room',
                    title=row[0],
                    location=row[0],
                    floor=row[1],
                    description=row[2],
                    notes=row[7],
                )
            )

        for row in people_rows[1:]:
            if len(row) < 8 or not row[0]:
                continue
            entries.append(
                DirectoryEntry(
                    kind='person',
                    title=row[0],
                    location=row[3],
                    floor=row[4],
                    description=row[1],
                    notes=row[5],
                )
            )

        for row in named_rows[1:]:
            if len(row) < 6 or not row[0]:
                continue
            entries.append(
                DirectoryEntry(
                    kind='space',
                    title=row[0],
                    location=row[1],
                    floor=row[2],
                    description=row[3],
                    notes='',
                )
            )

        return entries

    def _default_workbook_path(self) -> str:
        candidates = []

        try:
            share_dir = Path(get_package_share_directory('milton_final_project'))
            candidates.append(share_dir / 'data' / 'seic_public_directory.xlsx')
        except Exception:
            pass

        here = Path(__file__).resolve()
        candidates.append(here.parents[3] / 'data' / 'seic_public_directory.xlsx')
        candidates.append(here.parents[2] / 'data' / 'seic_public_directory.xlsx')

        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        return str(candidates[0])

    def _sheet_rows(self, workbook, sheet_file: str, shared_strings, namespace):
        root = ET.fromstring(workbook.read(f'xl/worksheets/{sheet_file}'))
        rows = []

        for row in root.findall('a:sheetData/a:row', namespace):
            values = []
            for cell in row.findall('a:c', namespace):
                inline = cell.find('a:is', namespace)
                value = cell.find('a:v', namespace)
                cell_type = cell.attrib.get('t')

                if inline is not None:
                    text = ''.join(node.text or '' for node in inline.findall('.//a:t', namespace))
                    values.append(text)
                elif value is None:
                    values.append('')
                elif cell_type == 's':
                    values.append(shared_strings[int(value.text)])
                else:
                    values.append(value.text or '')
            rows.append(values)

        return rows

    def _normalize_query(self, text: str) -> str:
        text = text.strip()
        text = re.sub(
            r'\b(can you|please|help me|i need|take me to|where is|find|locate|go to|office|room|lab|location)\b',
            ' ',
            text,
            flags=re.IGNORECASE,
        )
        text = re.sub(r'\s+', ' ', text).strip(' .?')

        room_match = re.search(r'(?:seic\s*)?(\d{3}[a-z]?)', text, flags=re.IGNORECASE)
        if room_match:
            return f'SEIC {room_match.group(1).upper()}'

        return text

    def _score_entry(self, query: str, entry: DirectoryEntry) -> float:
        query_norm = self._tokenize(query)
        if not query_norm:
            return 0.0

        candidates = [
            entry.title,
            entry.location,
            entry.description,
            entry.notes,
        ]
        candidate_text = ' '.join(part for part in candidates if part)
        candidate_norm = self._tokenize(candidate_text)

        if query_norm == self._tokenize(entry.title) or query_norm == self._tokenize(entry.location):
            return 1.0

        if query_norm in candidate_norm:
            return 0.94

        query_tokens = set(query_norm.split())
        candidate_tokens = set(candidate_norm.split())
        overlap = len(query_tokens & candidate_tokens) / max(1, len(query_tokens))
        similarity = SequenceMatcher(None, query_norm, candidate_norm).ratio()
        fuzzy_token_similarity = self._best_token_alignment(query_norm.split(), candidate_norm.split())
        return max(
            similarity * 0.55 + overlap * 0.15 + fuzzy_token_similarity * 0.30,
            overlap * 0.9,
            fuzzy_token_similarity * 0.92,
        )

    def _tokenize(self, text: str) -> str:
        text = text.lower()
        text = text.replace('&', ' and ')
        text = re.sub(r'[^a-z0-9]+', ' ', text)
        return re.sub(r'\s+', ' ', text).strip()

    def _best_token_alignment(self, query_tokens, candidate_tokens) -> float:
        if not query_tokens or not candidate_tokens:
            return 0.0

        scores = []
        for query_token in query_tokens:
            token_scores = [
                SequenceMatcher(None, query_token, candidate_token).ratio()
                for candidate_token in candidate_tokens
            ]
            scores.append(max(token_scores) if token_scores else 0.0)

        return sum(scores) / len(scores)
