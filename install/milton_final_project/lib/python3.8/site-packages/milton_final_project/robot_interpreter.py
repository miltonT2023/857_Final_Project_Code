import re
import shutil
import subprocess
from typing import Optional


class RobotInterpreter:
    def __init__(self, model: str = 'llama3.2:latest', timeout_sec: float = 5.0):
        self.model = model
        self.timeout_sec = timeout_sec
        self.ollama_path = shutil.which('ollama')

    def extract_target(self, user_text: str) -> str:
        cleaned = ' '.join(user_text.strip().split())
        if not cleaned:
            return ''

        heuristic = self._heuristic_extract(cleaned)
        if self._should_skip_llm(cleaned, heuristic):
            return heuristic

        llm_target = self._llm_extract(cleaned)

        if llm_target:
            return llm_target
        return heuristic

    def _should_skip_llm(self, original_text: str, heuristic: str) -> bool:
        if not heuristic:
            return False

        if heuristic.upper().startswith('SEIC '):
            return True

        word_count = len(heuristic.split())
        if word_count <= 4:
            return True

        original_word_count = len(original_text.split())
        return original_word_count <= 5

    def _heuristic_extract(self, text: str) -> str:
        room_match = re.search(r'(?:seic\s*)?(\d{3}[a-z]?)', text, flags=re.IGNORECASE)
        if room_match:
            return f'SEIC {room_match.group(1).upper()}'

        stripped = re.sub(
            r'\b(hi|hello|hey|can you|could you|please|help me|i need|i am looking for|looking for|take me to|where is|find|locate|go to|room|office)\b',
            ' ',
            text,
            flags=re.IGNORECASE,
        )
        stripped = re.sub(r'\s+', ' ', stripped).strip(' .?')
        return stripped or text

    def _llm_extract(self, text: str) -> Optional[str]:
        if not self.ollama_path:
            return None

        prompt = (
            'Extract only the destination, room, office, person, or named place from the sentence. '
            'Return only the target text with no explanation.\n'
            f'Input: {text}\n'
            'Target:'
        )

        try:
            result = subprocess.run(
                [self.ollama_path, 'run', self.model, prompt],
                check=False,
                capture_output=True,
                text=True,
                timeout=self.timeout_sec,
            )
        except (OSError, subprocess.SubprocessError, subprocess.TimeoutExpired):
            return None

        if result.returncode != 0:
            return None

        output = self._clean_output(result.stdout)
        if not output:
            return None
        return output

    def _clean_output(self, text: str) -> str:
        text = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', text)
        text = ' '.join(text.replace('\r', ' ').split())
        if 'Target:' in text:
            text = text.split('Target:', 1)[-1].strip()
        text = text.strip(' "\'')
        if len(text.split()) > 8:
            return ''
        return text
