# cyberdog visual programming documentation

## 概述
> 为 cyberdog 可视化编程项目提供文档。

## 安装
```
sudo apt-get install doxygen	            // 安装doxygen
sudo apt-get install texlive-full	        // latex转pdf相关
sudo apt-get install latex-cjk-chinese*		// cjk，中文依赖
sudo apt-get install cjk-latex            // cjk
```

## 生成
### 1. 生成 html+latex
```
cd <xxx/interaction/cyberdog_vp/cyberdog_vp/doc/doxygen>
doxygen cyberdog_vp.doxygen
```
> 上述指令会在 cyberdog_vp.doxygen 文件 OUTPUT_DIRECTORY 字段所设路径下生成 html+latex+man文件夹，其中：
>> html 下 index.html 为网页版文档；
>> latex 下为 latex 所需文件。

### 2. 生成 pdf
```
cd <cyberdog_vp.doxygen->OUTPUT_DIRECTORY->latex>
// xxx/build/cyberdog_vp/doxygen/latex/
// 对当前路径下 refman.tex 做一些微调
// make 前先阅读下述微调 refman.tex 的内容（涉及纸张大小、方向、字体等）。
cd ~/ros2/cybergog/build/cyberdog_vp/doxygen/latex
make && cp ./refman.pdf ../../../../src/interaction/cyberdog_vp/cyberdog_vp/pdf/CyberdogRobotHelpDocumentation.pdf
```
> 上述指令会在 cyberdog_vp.doxygen 文件 OUTPUT_DIRECTORY 字段所设路径下 latex 文件夹生成名为 refman.pdf 的文件（目标文件），并复制到指定目录。

### 3. 微调 refman.tex
#### 3.1 页面方向
>> 如果需要生成横向页面的 pdf 文档，则需要先修改上述路径下的 refman.tex 文件如下内容（大约59行）后，再 make：
```
% Page & text layout
\usepackage{geometry}
\geometry{%
  a4paper,%
  top=2.5cm,%
  bottom=2.5cm,%
  left=2.5cm,%
  right=2.5cm,%           // 追加','
  landscape=true%         // 追加行
}
```
#### 3.2 字体
>>如果需要修改字体信息（使其使用 CJKutf8 中文依赖，避免繁体字或乱码），则同样先修改上述路径下的 refman.tex 文件中下述内容（大约151行）后，再 make：
```
\usepackage{CJKutf8}      // 增加行
\begin{document}
\begin{CJK}{UTF8}{gbsn}   // 修改{min} -> {gbsn}
...
```
#### 3.3 索引
>>如果需要修取消文件末尾索引，则同样先修改上述路径下的 refman.tex 文件中下述内容（倒数第4行）后，再 make：
```
% \printindex             // 注释该行

\end{CJK}
\end{document}
```
#### 3.4 制作人
>> 如果需要修改制作者等信息，则同样先修改上述路径下的 refman.tex 文件中和'制作者'相关的内容后，再 make：
```
...
\fancyfoot[RE]{\fancyplain{}{\bfseries\scriptsize 尚子涵（Doxygen 1.8.17 2023.04.07） }}
\fancyfoot[LO]{\fancyplain{}{\bfseries\scriptsize 小米机器人事业部 }}
...
{\large 制作者 小米机器人事业部-尚子涵（Doxygen 1.8.17 2023.04.07）}\\
...
```
