from xgoedu import XGOEDU


class RuiKonSound:
    """区域物品音频播放模块，支持 A/B/C/D 四个区域和九个物品。"""

    ITEMS = [
        'apple',
        'clothe',
        'banana',
        'orange',
        'toothbrush',
        'paper',
        'Airconditioner',
        'TV',
        'fridge'
    ]

    AREA_LETTERS = ['A', 'B', 'C', 'D']
    ITEM_NAME_MAP = {item.lower(): item for item in ITEMS}

    @staticmethod
    def _normalize_area(area):
        if not isinstance(area, str):
            raise TypeError('area 必须是字符串，例如 A/B/C/D')

        area = area.strip().upper()
        if area not in RuiKonSound.AREA_LETTERS:
            raise ValueError("area 必须是 A/B/C/D 之一")
        return area

    @staticmethod
    def _normalize_item(item):
        if isinstance(item, int):
            if item < 0 or item >= len(RuiKonSound.ITEMS):
                raise ValueError(f'item 索引必须在 0 到 {len(RuiKonSound.ITEMS) - 1} 之间')
            return RuiKonSound.ITEMS[item]

        if not isinstance(item, str):
            raise TypeError('item 必须是字符串或整数索引')

        item_name = item.strip()
        if not item_name:
            raise ValueError('item 不能为空字符串')

        normalized = item_name.lower()
        if normalized not in RuiKonSound.ITEM_NAME_MAP:
            raise ValueError(
                f"不支持的物品: {item_name}. 支持的物品: {', '.join(RuiKonSound.ITEMS)}"
            )
        return RuiKonSound.ITEM_NAME_MAP[normalized]

    @staticmethod
    def play(area, item):
        """播放指定区域和物品对应的音频，例如 A_banana.wav。"""
        area = RuiKonSound._normalize_area(area)
        item_name = RuiKonSound._normalize_item(item)
        filename = f"{area}_{item_name}.wav"

        xgo = XGOEDU()
        xgo.xgoSpeaker(filename)

    @staticmethod
    def play_by_index(area, item_index):
        """按物品索引播放音频。索引范围为 0-8。"""
        area = RuiKonSound._normalize_area(area)
        item_name = RuiKonSound._normalize_item(item_index)
        RuiKonSound.play(area, item_name)

    @staticmethod
    def get_supported_items():
        return list(RuiKonSound.ITEMS)

    @staticmethod
    def get_supported_areas():
        return list(RuiKonSound.AREA_LETTERS)


if __name__ == '__main__':
    print('支持的区域:', RuiKonSound.get_supported_areas())
    print('支持的物品:', RuiKonSound.get_supported_items())
    print('示例调用: RuiKonSound.play("A", "banana")')
