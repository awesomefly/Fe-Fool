# 象棋棋子位置识别

## 一、统一标准坐标体系（红方视角，行业通用）

中国象棋红方视角的标准坐标规则（完全匹配棋盘物理标注）：

- **列（路）**：棋盘底部从左到右标注 9 8 7 6 5 4 3 2 1 → 对应红方的9路（最左）到1路（最右）

- **行（线）**：从红方底线（棋盘最下方）为1线，向上依次递增，黑方底线（棋盘最上方）为10线。

## 二、JSON返回格式（严格约束）

**重要提示**：请直接返回纯JSON数据，**不要**添加任何markdown代码块标记（如```json）、注释或其他格式化文本。

返回格式：
```json
{
  "pieces": [
    [x1, y1, c1, n1],
    [x2, y2, c2, n2],
    ...
  ],
  "analysis": {
    "advantage": "balanced",
    "key_threats": [],
    "suggested_moves": []
  }
}
```

其中：
- **x**: 横坐标（1-9，对应1路到9路）
- **y**: 纵坐标（1-10，对应1线到10线）
- **c**: 分类（red/black）
- **n**: 名称（hongbing、hongpao、heiche、heima、heixiang、heishi、heijiang、heipao、heibing、 hongche、 hongma、hongxiang、hongshi、hongshuai）

## 三、要求

### 1. 棋子识别
- 准确识别每个棋子的类型和颜色
- 精确计算每个棋子的坐标位置
- 区分红方和黑方

### 2. 棋盘状态
- 评估整体局势（红优/黑优/均势）
- 识别关键位置的争夺情况

### 3. 威胁分析
- 识别直接威胁（被吃的棋子）
- 发现潜在威胁（下一步可能被吃）
- 指出将军状态（如果有）

### 4. 走法建议
- 无特殊说明，默认轮到黑方走，为黑方提供3-5个合理的走法建议
- 每个建议包含：棋子、起始位置、目标位置、理由
- 按优先级排序

### 5. 特殊处理
- 如果棋盘不完整或图片模糊，请说明
- 如果无法确定某些棋子的位置，请标注"uncertain"
- 对于边界位置的棋子，请仔细确认坐标

## 四、示例输出（仅作格式参考）

注意：以下示例仅展示格式，实际坐标需根据图片确定：

```json
{
  "pieces": [
    [5, 1, "red", "hongbing"],
    [4, 1, "black", "heishi"],
    [6, 1, "black", "heishi"],
    [3, 1, "black", "heixiang"],
    [7, 1, "black", "heixiang"],
    [2, 1, "black", "heima"],
    [8, 1, "black", "heima"],
    [1, 1, "black", "heiche"],
    [9, 1, "black", "heiche"],
    [2, 3, "red", "hongpao"],
    [8, 3, "red", "hongpao"],
    [1, 4, "red", "hongbing"],
    [3, 4, "red", "hongbing"],
    [5, 4, "red", "hongbing"],
    [7, 4, "red", "hongbing"],
    [9, 4, "red", "hongbing"],
    [5, 10, "black", "heijiang"],
    [4, 10, "black", "heishi"],
    [6, 10, "black", "heishi"],
    [3, 10, "black", "heixiang"],
    [7, 10, "black", "heixiang"],
    [2, 10, "black", "heima"],
    [8, 10, "black", "heima"],
    [1, 10, "black", "heiche"],
    [9, 10, "black", "heiche"],
    [2, 8, "black", "heipao"],
    [8, 8, "black", "heipao"],
    [1, 7, "black", "heibing"],
    [3, 7, "black", "heibing"],
    [5, 7, "black", "heibing"],
    [7, 7, "black", "heibing"],
    [9, 7, "black", "heibing"]
  ],
  "analysis": {
    "advantage": "balanced",
    "key_threats": [],
    "suggested_moves": [
      {
        "piece": "hongpao",
        "from": [2, 3],
        "to": [5, 3],
        "reason": "控制中路，威胁黑方",
        "priority": 1
      }
    ]
  }
}
```

## 五、坐标参考图

```
黑方
10 ┏━┯━┯━┯━┯━┯━┯━┯━┓
9  ┠─┼─┼─┼─┼─┼─┼─┼─┨
8  ┠─┼─┼─┼─┼─┼─┼─┼─┨
7  ┠─┼─┼─┼─┼─┼─┼─┼─┨
6  ┠─┼─┼─┼─┼─┼─┼─┼─┨
5  ┠─┼─┼─┼─┼─┼─┼─┼─┨
4  ┠─┼─┼─┼─┼─┼─┼─┼─┨
3  ┠─┼─┼─┼─┼─┼─┼─┼─┨
2  ┠─┼─┼─┼─┼─┼─┼─┼─┨
1  ┗━┷━┷━┷━┷━┷━┷━┷━┛
   9 8 7 6 5 4 3 2 1
   红方
```

## 六、重要提醒

**再次强调**：请直接返回纯JSON字符串，不要添加任何额外的格式化、注释或说明文字。返回的数据应该可以直接被JSON解析器解析。