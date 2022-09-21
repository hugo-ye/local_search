#! /bin/sh


N=50
while  [ $N  -le  1000 ]
do
	./nqueens $N 1000 300 0.1 3 1325772160

	N=`expr $N + 50`
done
