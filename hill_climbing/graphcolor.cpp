#include <naxos.h>
#include <localS.h>

#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace std;
using namespace naxos;
using namespace localS;


int  main (int argc, char *argv[])
{
	try {

		if ( argc == 1 ) { cerr << "USAGE: N Pr  stateRepeats avoidAttempts walkProb tabuTenure seed" << endl; exit(1); }

		unsigned long stateRepeats = (argc > 3) ? atol(argv[3]) : 5;
		unsigned long avoidAttempts = (argc > 4) ? atol(argv[4]) : 0;
		double walkProb = (argc > 5) ? atof(argv[5]) : 0;
		unsigned long tabuTenure = (argc > 6) ? atol(argv[6]) : 0;
		unsigned long seed = (argc > 7) ? atol(argv[7]) : time(NULL);


		// number of nodes
		int  N = (argc > 1) ? atoi(argv[1]) : 9;

		// probability of edge existence between two nodes (%)
		int Pr = (argc > 2) ? atoi(argv[2]) : 25;

		int i,j;


		// construct a random graph
		srand(time(NULL));
		bool **graph = new bool*[N];
		for(i=0; i<N; ++i)
			graph[i] = new bool[N];
		for(i=0; i<N; ++i)
			for(j=0; j<=i; ++j)
				if (i > j  &&  rand()%100 <= Pr)
					graph[i][j] = true;
				else
					graph[i][j] = false;



		int k=1;
		//while(1) {	// is the graph k-colorable?
			LsProblemManager  pm( tabuTenure, seed );

			// HILL CLIMBING //
			MaxConflictingVariable 		selectVariable( pm );
			//MinConflictingVariable 	selectVariable( pm );
			//FirstVariable 		selectVariable( pm );
			//RandomVariable 		selectVariable( pm );
			//BiggestDomainVariable 	selectVariable( pm );
			//SmallestDomainVariable 	selectVariable( pm );
			MinConflictingValue 		selectValue( pm );
			//RandomValue 			selectValue( pm );
			// NOTE: Must be used together //
			//BestImprovementVariable 	selectVariable( pm );
			//BestImprovementValue 		selectValue( pm, selectVariable );
			LsProblemManager::HillConfiguration conf( &selectVariable, &selectValue, stateRepeats, avoidAttempts, walkProb );

			// PROBLEM STATEMENT //
			NsIntVarArray Nodes;

			for(i=0; i<N; ++i)
				Nodes.push_back( NsIntVar(pm, 0, k) );

			for(i=0; i<N; ++i)
				for(j=0; j<i; ++j)
					if(graph[i][j])
						pm.add( Nodes[i] != Nodes[j] );

			// LABELING //
			pm.label(Nodes, &conf);

			// SOLVING //
			pm.configuration( cout );
			//while (true)
			{
				pm.nextSolution();
				pm.solutionToString( cout );
				pm.statistics( cout );
			}

			k++;
		//}

    } catch (exception& exc)  {
	cerr << exc.what() << "\n";

    } catch (...)  {
	cerr << "Unknown exception" << "\n";
    }
}
