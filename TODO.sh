#!/bin/sh
grep -n -e FIXME: -e TODO: *.{cpp,h} | sed -e 's/:[[:space:]]*/:	/'
