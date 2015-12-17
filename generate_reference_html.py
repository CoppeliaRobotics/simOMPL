#!/usr/bin/env python3

from lxml import etree
from sys import argv

doc = etree.parse(argv[1])

print('''<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0 Strict//EN">
<html>

<head>
<meta http-equiv="Content-Language" content="en-us">
<title>API Functions</title>
<link rel="stylesheet" type="text/css" href="../../helpFiles/style.css">
</head>

<body>

<div align="center">
<table class=allEncompassingTable >
 <tr>
  <td >

<h1>OMPL Plugin API reference</h1>

<p class=infoBox>The list of API functions below allows you to define and solve a motion planning problem with OMPL.</p>

''')

def formatparams(s):
    if not s: return ''
    def formatparam(s):
        return '<div><strong>{}</strong>: {}</div>'.format(*s.split(': ', 1))
    return '\n'.join(map(formatparam, s.split('|')))

for cmd in sorted(doc.findall('command'), key=lambda x: x.get('name')):
    d=dict(
        fn=cmd.get('name'),
        syn=cmd.find('synopsis').text,
        descr=cmd.find('description').text,
        params=formatparams(cmd.find('params').text),
        ret=formatparams(cmd.find('return').text)
    )
    if not d['descr'] and not d['return'] and not d['params']: continue
    print('''
<h3 class=subsectionBar><a name="{fn}" id="{fn}"></a>{fn}</h3>
<table class=apiTable>
<tr class=apiTableTr>
<td class=apiTableLeftDescr>
Description
</td>
<td class=apiTableRightDescr>{descr}<br></td>
</tr>
<tr class=apiTableTr>
<td class=apiTableLeftCSyn>C synopsis</td>
<td class=apiTableRightCSyn>-</td>
</tr>
<tr class=apiTableTr>
<td class=apiTableLeftCParam>C parameters</td>
<td class=apiTableRightCParam>-</td>
</tr>
<tr class=apiTableTr>
<td class=apiTableLeftCRet>C return value</td>
<td class=apiTableRightCRet>-</td>
</tr>
<tr class=apiTableTr>
<td class=apiTableLeftLSyn>Lua synopsis</td>
<td class=apiTableRightLSyn>{syn}<br></td>
</tr>
<tr class=apiTableTr>
<td class=apiTableLeftLParam>Lua parameters</td>
<td class=apiTableRightLParam>{params}</td>
</tr>
<tr class=apiTableTr>
<td class=apiTableLeftLRet>Lua return values</td>
<td class=apiTableRightLRet>{ret}</td>
</tr>
</table>
<br>
'''.format(**d))

print('''
<br>
<br>
</td>
</tr>
</table>
</div>


</body>

</html>
''')
