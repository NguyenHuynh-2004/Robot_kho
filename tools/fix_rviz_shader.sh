#!/bin/bash
# Sua loi do cua RViz khi hien ban do tren WSL (Mesa):
#   [ERROR] ... rviz/glsl120/indexed_8bit_image.frag  GLSL link result :
#   active samplers with a different type refer to the same texture image unit
#
# Shader co sampler2D (anh ban do) va sampler1D (bang mau); luc lien ket ca hai
# mac dinh o texture unit 0 -> Mesa bao loi. Gan co dinh unit 0 / 1 (dung nhu
# file .material cua RViz: eight_bit_image=0, palette=1).
#
#   sudo tools/fix_rviz_shader.sh          # sua (giu ban goc .orig)
#   sudo tools/fix_rviz_shader.sh undo     # tra lai ban goc
# apt nang cap ros-jazzy-rviz-rendering se ghi de -> chay lai.
set -e
F=/opt/ros/jazzy/share/rviz_rendering/ogre_media/materials/glsl120/indexed_8bit_image.frag
[ "$(id -u)" = 0 ] || { echo "Can sudo: sudo $0 $*"; exit 1; }
[ -f "$F" ] || { echo "Khong thay $F"; exit 1; }

if [ "${1:-}" = undo ]; then
  [ -f "$F.orig" ] && cp "$F.orig" "$F" && echo "Da tra lai ban goc." || echo "Khong co ban goc $F.orig"
  exit 0
fi
if grep -q 'layout(binding' "$F"; then
  echo "Da sua tu truoc - khong lam gi."
  exit 0
fi
cp "$F" "$F.orig"
sed -i -e 's/^#version 120$/#version 130\n#extension GL_ARB_shading_language_420pack : require/' \
       -e 's/^uniform sampler2D eight_bit_image;/layout(binding = 0) uniform sampler2D eight_bit_image;/' \
       -e 's/^uniform sampler1D palette;/layout(binding = 1) uniform sampler1D palette;/' "$F"
grep -c 'layout(binding' "$F" | grep -q 2 || { cp "$F.orig" "$F"; echo "Sua that bai - da tra lai ban goc."; exit 1; }
echo "Da sua $F (ban goc: $F.orig). Mo lai RViz."
