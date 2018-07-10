#!/bin/sh
xsel -ob | sed -E \
  -e 's/@([a-zA-Z0-9_]*)/this->\1_/g' \
  -e 's/if (.*)$/if (\1) {/' \
  -e 's/else/} else {/' \
  -e 's/elsif/} else if/' \
  -e 's/end/}/' \
  -e '/[{}]/!s/([^ ])$/\1;/' \
| xsel -ib
