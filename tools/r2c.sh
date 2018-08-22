#!/bin/sh
xsel -ob | sed -E \
  -e 's/#/\/\//g' \
  -e 's/def (.*)\((.*)\)/void \1(\2) {/g' \
  -e 's/def (.*)$/void \1() {/g' \
  -e 's/@([a-zA-Z0-9_]*)/this->\1_/g' \
  -e 's/case (.*)$/switch (\1) {/' \
  -e 's/when (.*) then (.*)/case \1: \2; break/' \
  -e 's/else (.*)/default: \1; break/' \
  -e '/^ *if/s/if (.*)$/if (\1) {/' \
  -e 's/([^ ].*) if (.*)$/if (\2) { \1; }/' \
  -e 's/([^ ].*) unless (.*)$/if (!(\2)) { \1; }/' \
  -e '/^ *else/s/else/} else {/' \
  -e '/^ *elsif/s/elsif (.*)$/} else if (\1) {/' \
  -e '/^ *while/s/while (.*)$/while (\1) {/' \
  -e 's/end while (.*)$/} while (\1);/' \
  -e 's/([^ }].*) while (.*)$/while (\2) { \1; }/' \
  -e '/^ *begin$/s/begin/do {/' \
  -e 's/end/}/' \
  -e '/[{}]/!s/([^ ])$/\1;/' \
| xsel -ib
