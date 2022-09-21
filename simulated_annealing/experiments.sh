#! /bin/sh


N=25
while  [ $N  -le  300 ]
do
	echo

	attempts=1
	while  [ $attempts  -le  5 ]
	do
		./nqueens $N $attempts 2 0.1 3 1325772160

		attempts=`expr $attempts + 1`
	done

	N=`expr $N + 25`
done
