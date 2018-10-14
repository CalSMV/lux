# :%s/^[ ]\+/\t/g

CC=g++

src_dir = ./src
inc_dir = ./include

clean:
	rm -f *.o *.out
