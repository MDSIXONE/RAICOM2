from pathlib import Path
from typing import Optional
from xgoedu import XGOEDU

class Sound():
    @staticmethod
    def play_gauge_status(letter, status):
        """
        播放仪表盘状态语音

        Args:
            letter: 字母 ('A', 'B', 'C', 'D')
            status: 状态 ('High', 'Normal', 'Low')
        """
        XGO_EDU = XGOEDU()

        # 根据字母和状态播放相应语音
        if status == 'Low':
            if letter == 'A':
                XGO_EDU.xgoSpeaker("A_Low.wav")  # A区域仪表盘显示偏低，状态异常
            elif letter == 'B':
                XGO_EDU.xgoSpeaker("B_Low.wav")
            elif letter == 'C':
                XGO_EDU.xgoSpeaker("C_Low.wav")
            elif letter == 'D':
                XGO_EDU.xgoSpeaker("D_Low.wav")
        elif status == 'High':
            if letter == 'A':
                XGO_EDU.xgoSpeaker("A_High.wav")  # A区域仪表盘显示偏高，状态异常
            elif letter == 'B':
                XGO_EDU.xgoSpeaker("B_High.wav")
            elif letter == 'C':
                XGO_EDU.xgoSpeaker("C_High.wav")
            elif letter == 'D':
                XGO_EDU.xgoSpeaker("D_High.wav")
        elif status == 'Normal':
            if letter == 'A':
                XGO_EDU.xgoSpeaker("A_Normal.wav")  # A区域仪表盘显示正常
            elif letter == 'B':
                XGO_EDU.xgoSpeaker("B_Normal.wav")
            elif letter == 'C':
                XGO_EDU.xgoSpeaker("C_Normal.wav")
            elif letter == 'D':
                XGO_EDU.xgoSpeaker("D_Normal.wav")

    @staticmethod
    def _find_audio_file(filename: str) -> Optional[Path]:
        candidates = [
            Path(__file__).resolve().parent / filename,
            Path('/home/pi/Desktop/DOG1/lib') / filename,
            Path.cwd() / filename,
            Path(filename)
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate
        return None

    @staticmethod
    def play_helmet_warning(index: int):
        """
        播放安全帽缺失报警音频，按照数字文件名 1.wav、2.wav 等播放。
        """
        XGO_EDU = XGOEDU()
        filename = f"{index}.wav"
        file_path = Sound._find_audio_file(filename)
        if file_path is not None:
            XGO_EDU.xgoSpeaker(str(file_path))
        else:
            print(f"警告：未找到语音文件 {filename}，直接播放 {filename}")
            XGO_EDU.xgoSpeaker(filename)

    @staticmethod
    def play_number(number: int):
        """
        播放数字语音文件，例如 0.wav、1.wav、2.wav ...
        """
        XGO_EDU = XGOEDU()
        filename = f"{number}.wav"
        file_path = Sound._find_audio_file(filename)
        if file_path is not None:
            XGO_EDU.xgoSpeaker(str(file_path))
        else:
            print(f"警告：未找到数字语音文件 {filename}，直接播放 {filename}")
            XGO_EDU.xgoSpeaker(filename)
