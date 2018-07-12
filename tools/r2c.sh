#!/bin/sh
xsel -ob | sed -E \
  -e 's/@([a-zA-Z0-9_]*)/this->\1_/g' \
  -e 's/case (.*)$/switch (\1) {/' \
  -e 's/when (.*) then (.*)/case \1: \2; break/' \
  -e 's/else (.*)/default: \1; break/' \
  -e 's/if (.*)$/if (\1) {/' \
  -e 's/else/} else {/' \
  -e 's/elsif/} else if/' \
  -e 's/end/}/' \
  -e '/[{}]/!s/([^ ])$/\1;/' \
| xsel -ib
