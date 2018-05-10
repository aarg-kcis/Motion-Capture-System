#!/bin/bash
rm ID.txt
h=$1
v=$2
id=0
for j in $(seq 0 $(($v-1)));do
for i in $(seq 0 $(($h-1)));do
convert -size 2970x2100 xc:white -density 10x10 -units pixelspercentimeter -fill white res.png
echo -e Creating actual robot patterns $(($i+$j*$h)) of  $(($v*$h)) \\r 
sj=300/$v
si=350/$h
x=1485 #$(($i*400+650)); 
y=1050 #$(($j*400+250));
ix=$((($j*10+10)*$sj/10+35));
iy=$((($i*10+10)*$si/10+35)); 
echo $x
echo $ix
echo $y
echo $iy
convert res.png \
-fill white -stroke black -draw "ellipse $x,$y 1000,1000 0,360" \
-fill black -stroke none -draw "ellipse $x,$y 650,800 0,360" \
-fill white -stroke none -draw "ellipse $x,$(($y+50)) $ix,$iy 0,360" \
res$i$j.png
convert res$i$j.png -density 100x100 res_$i\_$j.pdf
echo $ix $iy
r0=$(echo $ix|awk '{print $1/650}')
r1=$(echo $iy|awk '{print $1/800}')
echo $id $r1 $r0 >>ID.txt
id=$(($id+1))
rm res.png
done
done
# convert res.png -density 100x100 res_$h\_$v.pdf
