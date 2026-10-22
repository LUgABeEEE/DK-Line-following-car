# 从厂商例程 oledfont.h 提取 ASCII 点阵，输出规范的 C 头文件。
#
# 关键结论：两张表的【数组下标 == ASCII码 - 0x20】，末尾各有一个非字符条目。
# 所以按索引归档是最可靠的，注释只用来做交叉校验。
#
# 原厂数据的问题（脚本会报告）：
#   - asc2_0806 第 91 项注释为 "horiz lines"，非字符，丢弃；
#   - asc2_0806 第 60 项（反斜杠）注释被笔误写成 "55"（本意 5C）、
#     第 64 项（反引号）注释被误写成 "'"，都只影响注释不影响数据；
#   - asc2_1608 第 92 项注释为 "|"，实际应为反引号（数据正确）。
import re, glob, sys

cands = [c for c in glob.glob("/home/JiangyaF/STMpro/linefllowing_car/.oled/**/oledfont.h",
                              recursive=True) if "F103C8T6" in c]
if not cands:
    sys.exit("找不到 oledfont.h")
src = open(cands[0], "rb").read().decode("gbk", errors="replace")

def parse(name, per_char):
    m = re.search(re.escape(name) + r"\[\][^=]*=\s*\{(.*?)\n\};", src, re.S)
    if not m:
        sys.exit("找不到 " + name)

    rows, notes = [], []
    for order, line in enumerate(m.group(1).replace("\r", "").split("\n")):
        vals = [int(v, 16) for v in re.findall(r"0x([0-9A-Fa-f]{1,2})", line.split("/*")[0].split("//")[0])]
        if not vals:
            continue
        if len(vals) != per_char:
            notes.append(f"第 {order} 项字节数 {len(vals)} != {per_char}，丢弃")
            continue

        tag = re.search(r"//\s*(.+?)\s*$", line)
        tag = tag.group(1).strip() if tag else ""
        tag = re.sub(r'^/\*"?(.*?)"?,?\d*\*/$', r"\1", tag).strip()

        expect = chr(0x20 + order) if 0x20 + order <= 0x7E else None

        if expect is None:
            notes.append(f"第 {order} 项超出 ASCII 范围（注释 '{tag}'），丢弃")
            continue

        if tag != expect:
            notes.append(f"第 {order} 项注释 '{tag}' 与预期 '{expect}' 不符（数据仍按索引归档）")

        rows.append(vals)

    return rows, notes

r6,  n6  = parse("asc2_0806", 6)
r16, n16 = parse("asc2_1608", 16)

for nm, rows, notes in (("6x8", r6, n6), ("8x16", r16, n16)):
    print(f"{nm}: 归档 {len(rows)} 项")
    for x in notes:
        print("      -", x)

def pad(rows, per_char, count=95):
    out = list(rows[:count])
    while len(out) < count:
        out.append([0] * per_char)
    return out

t6, t16 = pad(r6, 6), pad(r16, 16)

for nm, tb in (("6x8", t6), ("8x16", t16)):
    blank = [chr(0x20 + i) for i, r in enumerate(tb) if all(b == 0 for b in r)]
    print(f"  {nm} 全空白字符: {blank if blank else '无'}")
    # 抽样验证几个关键字符的形状是否合理（非零字节数）
    for c in "0AZaz. ":
        i = ord(c) - 0x20
        nz = sum(1 for b in tb[i] if b)
        print(f"      '{c}' -> 索引{i:2d} 非零字节 {nz}/{len(tb[i])}")

def emit(name, table, per_char, comment):
    out = [f"/* {comment} */",
           f"static const uint8_t {name}[{len(table)}][{per_char}] =", "{"]
    for idx, row in enumerate(table):
        ch = chr(0x20 + idx)
        label = {" ": "sp", "\\": "backslash", "'": "quote", '"': "dquote"}.get(ch, ch)
        body = ", ".join(f"0x{b:02X}" for b in row)
        out.append(f"    {{ {body} }},   /* {idx:2d}  0x{0x20+idx:02X}  '{label}' */")
    out += ["};", ""]
    return "\n".join(out)

with open("/home/JiangyaF/STMpro/linefllowing_car/.oled/gen/font_tables.h", "w") as fp:
    fp.write("""/*
 * OLED ASCII 字体点阵表
 *
 * 本文件由 genfont.py 从厂商例程 oledfont.h 程序化提取生成，请勿手工编辑。
 * 覆盖可打印 ASCII 0x20(' ') ~ 0x7E('~')，共 95 个字符，
 * 下标恒等于 (ASCII码 - 0x20)。
 *
 * 原厂数据说明（不影响点阵内容，仅注释有误）：
 *   - asc2_0806 末尾多一个注释为 "horiz lines" 的非字符条目，已剔除；
 *   - asc2_0806 中反斜杠的注释被笔误写成 "55"（本意 5C），
 *     反引号的注释被误写成 "'"，点阵数据本身正确；
 *   - asc2_1608 中反引号的注释被写成 "|"，点阵数据本身正确。
 *
 * 数据组织方式：每个字符按【列】存放，每列 1 字节，位 0 在【上】。
 *   6x8 : 6 列 x 8 位   -> 每字符 6 字节，占 1 个 page
 *   8x16: 6 列 x 16 位  -> 每字符 16 字节，前 8 字节是上半页，后 8 字节是下半页
 */

#ifndef OLED_FONT_TABLES_H
#define OLED_FONT_TABLES_H

#include <stdint.h>

""")
    fp.write(emit("oled_font_6x8", t6, 6, "6x8 点阵，每字符 6 字节") + "\n")
    fp.write(emit("oled_font_8x16", t16, 16, "8x16 点阵，每字符 16 字节") + "\n")
    fp.write("#endif /* OLED_FONT_TABLES_H */\n")

print("已生成 font_tables.h")
