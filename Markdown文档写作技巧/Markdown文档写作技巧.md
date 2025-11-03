
---

# 📝 Markdown 常用写作技巧指南

> 简洁、美观、结构清晰的 Markdown 文档，让你的内容更专业。

---

## 📖 1. 标题（Title）

使用 `#` 表示不同层级的标题：

```markdown
# 一级标题
## 二级标题
### 三级标题
#### 四级标题
```

👉 **建议：**

* 一级标题仅用于文档名称；
* 二级、三级标题用于章节和小节。

---

## 🧩 2. 段落与换行

* 段落之间空一行
* 在行尾加两个空格或 `<br>` 强制换行

```markdown
这是第一行  
这是第二行
```

---

## 🗂️ 3. 列表（Lists）

### 🔸 无序列表

```markdown
- 苹果
- 香蕉
  - 小香蕉
  - 大香蕉
```

### 🔹 有序列表

```markdown
1. 第一步
2. 第二步
3. 第三步
```

---

## 💬 4. 引用（Quote）

用 `>` 表示引用：

```markdown
> 这是一个引用。
> 可以多行。
```

> 这是一个引用。
> 可以多行。

---

## 💡 5. 强调（Emphasis）

```markdown
*斜体* 或 _斜体_  
**加粗** 或 __加粗__  
***加粗斜体***  
~~删除线~~
```

👉 **显示效果：**
*斜体*、**加粗**、***加粗斜体***、~~删除线~~

---

## 🔗 6. 链接（Links）

```markdown
[百度一下](https://www.baidu.com)
<https://openai.com>
```

👉 [百度一下](https://www.baidu.com)

---

## 🖼️ 7. 图片（Images）

```markdown
![图片描述](https://example.com/image.png)
```

👉 ![图片描述](https://example.com/image.png)

---

## 🧮 8. 代码（Code）

### 行内代码：

\```markdown  
请使用 `printf()` 输出结果。  
\```

👉 请使用 `printf()` 输出结果。

### 代码块（多行）：

\```cpp  
#include \<iostream>  
int main()  
{  
std::cout << "Hello Markdown!" << std::endl;  
}  
\```

👉 显示为：

```cpp
#include <iostream>
int main() {
    std::cout << "Hello Markdown!" << std::endl;
}
```

---

## 📊 9. 表格（Tables）

```markdown  
| 姓名 | 年龄 | 职业 |
| ---- | ---- | ---- |
| 张三 | 25   | 工程师 |
| 李四 | 30   | 设计师 |
```

👉 显示为：

| 姓名 | 年龄 | 职业  |
| -- | -- | --- |
| 张三 | 25 | 工程师 |
| 李四 | 30 | 设计师 |

***可以用“:”来改变对齐方式***

---

## 📚 10. 任务清单（Task List）

```markdown
- [x] 已完成任务
- [ ] 待办任务
```

👉 显示为：

* [x] 已完成任务
* [ ] 待办任务

---

## 🧭 11. 分割线（Horizontal Rule）

```markdown
---
```

---

## 🧱 12. 提示块（在 GitHub / Docs 工具中常见）

部分渲染器支持：

```markdown
> 💡 **提示：** 可以用 emoji 提示重点。
> ⚠️ **警告：** 操作前请保存文件。
> ✅ **完成：** 任务已执行。
```

> 💡 **提示：** 可以用 emoji 提示重点。
> ⚠️ **警告：** 操作前请保存文件。
> ✅ **完成：** 任务已执行。

---

## 📎 13. 相对路径与文件引用

```markdown
[查看详细说明](docs/detail.md)
![示意图](images/diagram.png)
```

---

## ⚙️ 14. 数学公式（需渲染器支持，如 Typora / GitHub）

```markdown
行内公式：$E = mc^2$
块级公式：
$$
\nabla \cdot \vec{E} = \frac{\rho}{\varepsilon_0}
$$
```

---

## 🧭 15. 实用排版建议

✅ 建议：

* 英文与中文之间加空格，如 “使用 Markdown 编写文档”
* 保持行宽 < 120 字符，便于阅读
* 用 emoji 提高可读性
* 使用空行分隔段落

---
