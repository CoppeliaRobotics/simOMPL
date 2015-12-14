#!/usr/bin/env python3

import re
# collect macro definitions:
macrodefs={}
with open('v_repExtOMPL.cpp') as f:
    lastkey=None
    laststring=''
    for l in f:
        m=re.match('^\s*#define\s+([A-Za-z_][A-Za-z0-9_]*)\s+(.*)$',l)
        if lastkey:
            k=lastkey
            v=l
            lastkey=None
            string=laststring
            laststring=''
        elif not m:
            continue
        else:
            k=m.group(1)
            v=m.group(2)
            string=''
        # do one expansion pass:
        st=0
        ident=''
        for c in v:
            if st==0:
                if c=='"':
                    if ident and ident in macrodefs: string+=macrodefs[ident]
                    ident=''
                    st=1
                elif not c.isspace():
                    ident+=c
            elif st==1:
                if c=='"':
                    st=0
                elif c=='\\':
                    st=2
                else:
                    string+=c
            elif st==2:
                string+=c
                st=1
        if ident:
            if ident=='\\':
                laststring=string
                lastkey=k
            else:
                if ident in macrodefs: string+=macrodefs[ident]
        if lastkey is None:
            macrodefs[k]=string



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

for k in sorted(macrodefs):
    m=re.match('^LUA_(.*)_COMMAND$',k)
    if m:
        ik=m.group(1)
        if 'LUA_%s_NODOC'%ik in macrodefs: continue
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
'''.format(
        fn=macrodefs[k],
        syn=macrodefs.get('LUA_%s_APIHELP'%ik),
        descr=macrodefs.get('LUA_%s_DESCR'%ik,''),
        params=formatparams(macrodefs.get('LUA_%s_PARAMS'%ik,'')),
        ret=formatparams(macrodefs.get('LUA_%s_RET'%ik,''))
))

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
