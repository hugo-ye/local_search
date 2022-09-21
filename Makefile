####  COMPILING AND LINKING OPTIONS  ####

#  Uncomment the following line, for Sun compilation at di.uoa.gr domain.
#CPATH = /usr/sfw/bin/

#  Uncomment the following line, for gcc versions greater than 4.2.
#STANDARD = -std=c++0x

# Naxos Directory
ND = ../naxos/

CC = $(CPATH)g++
WFLAGS = -pedantic -Wall -W -Wshadow
CFLAGS = $(WFLAGS) $(STANDARD) -O

LD = $(CC)
LDFLAGS = -s

RM = /bin/rm -f

####  SOURCE AND OUTPUT FILENAMES  ####

HDRS = $(ND)naxos.h $(ND)internal.h $(ND)stack.h 	localS.h auxiliary.h mtrand.h md5.h
NOBJ = $(ND)local_search.o $(ND)problemmanager.o $(ND)expressions.o $(ND)var_constraints.o $(ND)array_constraints.o $(ND)intvar.o $(ND)bitset_domain.o
SRCS = localS.cpp mtrand.cpp md5.cpp

OBJS = $(SRCS:.cpp=.o)

.PHONY: all
all: $(OBJS)

####  BUILDING  ####

%.o :  %.cpp $(HDRS) %.h
	$(CC) $(CFLAGS) -I$(ND) -I. -c  $<

####  CLEANING UP  ####

TODEL = $(OBJS)

.PHONY: clean
clean :
	$(RM)  $(TODEL)
