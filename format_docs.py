#!/usr/bin/env python3

from lxml import etree

from sys import stdin
doc = etree.fromstring(stdin.read())

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
def stringify_children(node):
    from lxml.etree import tostring
    from itertools import chain
    parts = ([node.text] +
            list(chain(*([tostring(c, with_tail=False), c.tail] for c in node.getchildren()))) +
            [node.tail])
    # filter removes possible Nones in texts and tails
    return ''.join(filter(None, map(lambda x: x.decode('utf-8') if isinstance(x,bytes) else x, parts)))

def formatparams(s):
    if not s: return ''
    def formatparam(s):
        s=s.strip()
        return '<div><strong>{}</strong>: {}</div>'.format(*s.split(': ', 1)) if s else ''
    return '\n'.join(map(formatparam, s.split('|')))

for cmd in sorted(doc.findall('command'), key=lambda x: x.get('name')):
    d=dict(
        fn=cmd.get('name'),
        syn=stringify_children(cmd.find('synopsis')),
        descr=stringify_children(cmd.find('description')),
        params=formatparams(stringify_children(cmd.find('params'))),
        ret=formatparams(stringify_children(cmd.find('return')))
    )
    if not d['descr'].strip(): continue
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
