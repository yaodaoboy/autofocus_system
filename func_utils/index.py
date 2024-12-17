import ctypes

# 数字转十六进制
def to_hex_str(num):
    cha_dic: dict[int, str] = {10: 'a', 11: 'b', 12: 'c', 13: 'd', 14: 'e', 15: 'f'}
    hex_str = ""
    if num < 0:
        num = num + 2 ** 32
    while num >= 16:
        digit = num % 16
        hex_str = cha_dic.get(digit, str(digit)) + hex_str
        num //= 16
    hex_str = cha_dic.get(num, str(num)) + hex_str
    return hex_str

def decoding_char(c_ubyte_value):
    """ Decode byte characters to string (GBK fallback). """
    c_char_p_value = ctypes.cast(c_ubyte_value, ctypes.c_char_p)
    try:
        return c_char_p_value.value.decode('gbk')
    except UnicodeDecodeError:
        return str(c_char_p_value.value)

def txt_wrap_by(start_str, end, all):
    """ Extract text between two strings. """
    start = all.find(start_str)
    if start >= 0:
        start += len(start_str)
        end = all.find(end, start)
        if end >= 0:
            return all[start:end].strip()