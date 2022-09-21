####  COMPILING AND LINKING OPTIONS  ####

#  Uncomment the following line, for Sun compilation at di.uoa.gr domain.
#CPATH = /usr/sfw/bin/

#  Uncomment the following line, for gcc versions greater than 4.2.
#STANDARD = -std=c++0x

# Naxos Directory
ND = ../../naxos/
# Methods Directory
MD = ../

CC = $(CPATH)g++
WFLAGS = -pedantic -Wall -W -Wshadow
CFLAGS = $(WFLAGS) $(STANDARD) -O

LD = $(CC)
LDFLAGS = -s

RM = /bin/rm -f

####  SOURCE AND OUTPUT FILENAMES  ####

NQUEENS = nqueens

ALLPROGS = $(NQUEENS)

HDRS = $(ND)naxos.h $(ND)internal.h $(ND)stack.h 	$(MD)localS.h $(MD)auxiliary.h $(MD)mtrand.h $(MD)md5.h
NOBJ = $(ND)local_search.o $(ND)problemmanager.o $(ND)expressions.o $(ND)var_constraints.o $(ND)array_constraints.o $(ND)intvar.o $(ND)bitset_domain.o
MOBJ = $(MD)localS.o $(MD)md5.o $(MD)mtrand.o

.PHONY: all
all: $(ALLPROGS)

####  BUILDING  ####

$(NQUEENS) :  $(NQUEENS).o 
	$(LD) $(LDFLAGS) $(NOBJ) $(MOBJ) $(NQUEENS).o  -o $@

%.o :  %.cpp $(HDRS)
	$(CC) $(CFLAGS) -I$(ND) -I$(MD) -c  $<

####  CLEANING UP  ####

TODEL = $(ALLPROGS) $(ALLPROGS:=.o)

.PHONY: clean
clean :
	$(RM)  $(TODEL)
