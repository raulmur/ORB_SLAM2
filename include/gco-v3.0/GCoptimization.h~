/*
##############################################################################
#                                                                            #
#    GCoptimization - software for energy minimization with graph cuts       #
#                        Version 3.0                                         #
#    http://www.csd.uwo.ca/faculty/olga/software.html                        #
#                                                                            #
#    Copyright 2007-2010 Olga Veksler (olga@csd.uwo.ca)                      #
#                        Andrew Delong (andrew.delong@gmail.com)             #
#                                                                            #
##############################################################################

  C++ requires at least Visual C++ 2005 (VC8) or GCC 4.03. Supports 32 or 64-bit.
  See matlab/README.TXT for bundled MATLAB wrapper and its documentation.

  IMPORTANT:
  To use this software, YOU MUST CITE the following in any resulting publication:

    [1] Efficient Approximate Energy Minimization via Graph Cuts.
        Y. Boykov, O. Veksler, R.Zabih. IEEE TPAMI, 20(12):1222-1239, Nov 2001.

    [2] What Energy Functions can be Minimized via Graph Cuts?
        V. Kolmogorov, R.Zabih. IEEE TPAMI, 26(2):147-159, Feb 2004. 

    [3] An Experimental Comparison of Min-Cut/Max-Flow Algorithms for 
        Energy Minimization in Vision. Y. Boykov, V. Kolmogorov. 
        IEEE TPAMI, 26(9):1124-1137, Sep 2004.

  Furthermore, if you use the label cost feature (setLabelCost), you should cite

    [4] Fast Approximate Energy Minimization with Label Costs. 
        A. Delong, A. Osokin, H. N. Isack, Y. Boykov. In CVPR, June 2010. 

  This software can be used only for research purposes. For commercial purposes, 
  be aware that there is a US patent on the main algorithm itself:

        R. Zabih, Y. Boykov, O. Veksler,
        "System and method for fast approximate energy minimization via graph cuts",
        United Stated Patent 6,744,923, June 1, 2004

  Together with this library implemented by O. Veksler, we provide, with the 
  permission of the V. Kolmogorov and Y. Boykov, the following two libraries:

  1) energy.h
     Developed by V. Kolmogorov, this implements the binary energy minimization 
     technique described in [2] above. We use this to implement the binary 
     energy minimization step for the alpha-expansion and swap algorithms. 
     The graph construction provided by "energy.h" is more efficient than 
     the original graph construction for alpha-expansion described in [1].

     Again, this software can be used only for research purposes. IF YOU USE 
     THIS SOFTWARE (energy.h), YOU SHOULD CITE THE AFOREMENTIONED PAPER [2] 
     IN ANY RESULTING PUBLICATION.

  2) maxflow.cpp, graph.cpp, graph.h, block.h
     Developed by Y. Boykov and V. Kolmogorov while at Siemens Corporate Research,
     algorithm [3] was later reimplemented by V. Kolmogorov based on open publications
     and we use his implementation here with permission.

  If you use either of these libraries for research purposes, you should cite
  the aforementioned papers in any resulting publication.

##################################################################

    License & disclaimer.

    Copyright 2007-2010 Olga Veksler  <olga@csd.uwo.ca>
                        Andrew Delong <andrew.delong@gmail.com>

    This software and its modifications can be used and distributed for 
    research purposes only. Publications resulting from use of this code
    must cite publications according to the rules given above. Only
    Olga Veksler has the right to redistribute this code, unless expressed
    permission is given otherwise. Commercial use of this code, any of 
    its parts, or its modifications is not permited. The copyright notices 
    must not be removed in case of any modifications. This Licence 
    commences on the date it is electronically or physically delivered 
    to you and continues in effect unless you fail to comply with any of 
    the terms of the License and fail to cure such breach within 30 days 
    of becoming aware of the breach, in which case the Licence automatically 
    terminates. This Licence is governed by the laws of Canada and all 
    disputes arising from or relating to this Licence must be brought 
    in Toronto, Ontario.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
    A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

##################################################################
*/

#ifndef __GCOPTIMIZATION_H__
#define __GCOPTIMIZATION_H__
// Due to quiet bugs in function template specialization, it is not 
// safe to use earlier MS compilers. 
#if defined(_MSC_VER) && _MSC_VER < 1400
#error Requires Visual C++ 2005 (VC8) compiler or later.
#endif

#include <cstddef>
#include "energy.h"
#include "graph.cpp"
#include "maxflow.cpp"

/////////////////////////////////////////////////////////////////////
// Utility functions, classes, and macros
/////////////////////////////////////////////////////////////////////

class GCException {
public:
	const char* message;
	GCException( const char* m ): message(m) { }
	void Report();
};

#ifdef _WIN32
typedef __int64 gcoclock_t;
#else
#include <ctime>
typedef clock_t gcoclock_t;
#endif
extern "C" gcoclock_t gcoclock(); // fairly high-resolution timer... better than clock() when available
extern "C" gcoclock_t GCO_CLOCKS_PER_SEC; // this variable will stay 0 until gcoclock() is called for the first time

#ifdef _MSC_EXTENSIONS
#define OLGA_INLINE __forceinline
#else
#define OLGA_INLINE inline
#endif

#ifndef GCO_MAX_ENERGYTERM
#define GCO_MAX_ENERGYTERM 10000000  // maximum safe coefficient to avoid integer overflow
                                     // if a data/smooth/label cost term is larger than this,
                                     // the library will raise an exception
#endif

#if defined(GCO_ENERGYTYPE) && !defined(GCO_ENERGYTERMTYPE)
#define GCO_ENERGYTERMTYPE GCO_ENERGYTYPE
#endif
#if !defined(GCO_ENERGYTYPE) && defined(GCO_ENERGYTERMTYPE)
#define GCO_ENERGYTYPE GCO_ENERGYTERMTYPE
#endif


/////////////////////////////////////////////////////////////////////
// GCoptimization class
/////////////////////////////////////////////////////////////////////
class LinkedBlockList;

class GCoptimization
{
public: 
#ifdef GCO_ENERGYTYPE
	typedef GCO_ENERGYTYPE EnergyType;
	typedef GCO_ENERGYTERMTYPE EnergyTermType;
#else
#ifdef GCO_ENERGYTYPE32
	typedef int EnergyType;        // 32-bit energy total
#else
	typedef long long EnergyType;  // 64-bit energy total
#endif
	typedef int EnergyTermType;    // 32-bit energy terms
#endif
	typedef Energy<EnergyTermType,EnergyTermType,EnergyType> EnergyT;
	typedef EnergyT::Var VarID;
	typedef int LabelID;                     // Type for labels
	typedef VarID SiteID;                    // Type for sites
	typedef EnergyTermType (*SmoothCostFn)(SiteID s1, SiteID s2, LabelID l1, LabelID l2);
	typedef EnergyTermType (*DataCostFn)(SiteID s, LabelID l);
	typedef EnergyTermType (*SmoothCostFnExtra)(SiteID s1, SiteID s2, LabelID l1, LabelID l2,void *);
	typedef EnergyTermType (*DataCostFnExtra)(SiteID s, LabelID l,void *);

	GCoptimization(SiteID num_sites, LabelID num_labels);
	virtual ~GCoptimization();

	// Peforms expansion algorithm. Runs the number of iterations specified by max_num_iterations 
	// If no input specified,runs until convergence. Returns total energy of labeling. 
	EnergyType expansion(int max_num_iterations=-1);

	// Peforms  expansion on one label, specified by the input parameter alpha_label 
	bool alpha_expansion(LabelID alpha_label);

	// Peforms swap algorithm. Runs it the specified number of iterations. If no  
	// input is specified,runs until convergence                                  
	EnergyType swap(int max_num_iterations=-1);

	// Peforms  swap on a pair of labels, specified by the input parameters alpha_label, beta_label 
	void alpha_beta_swap(LabelID alpha_label, LabelID beta_label);

	// Peforms  swap on a pair of labels, specified by the input parameters alpha_label, beta_label 
	// only on the sitess in the specified arrays, alphaSites and betaSitess, and the array sizes  
	// are, respectively, alpha_size and beta_size                                                  
	void alpha_beta_swap(LabelID alpha_label, LabelID beta_label, SiteID *alphaSites, 
		                 SiteID alpha_size, SiteID *betaSites, SiteID beta_size);

	struct DataCostFunctor;      // use this class to pass a functor to setDataCost 
	struct SmoothCostFunctor;    // use this class to pass a functor to setSmoothCost 

	// Set cost for all (SiteID,LabelID) pairs. Default data cost is all zeros.
	void setDataCost(DataCostFn fn);
	void setDataCost(DataCostFnExtra fn, void *extraData);
	void setDataCost(EnergyTermType *dataArray);
	void setDataCost(SiteID s, LabelID l, EnergyTermType e); 
	void setDataCostFunctor(DataCostFunctor* f);
	struct DataCostFunctor {
		virtual EnergyTermType compute(SiteID s, LabelID l) = 0;
	};
	// Set cost of assigning 'l' to a specific subset of sites.
	// The sites are listed as (SiteID,cost) pairs.
	struct SparseDataCost {
		SiteID site;
		EnergyTermType cost;
	};
	void setDataCost(LabelID l, SparseDataCost *costs, SiteID count);

	// Set cost for all (LabelID,LabelID) pairs; the actual smooth cost is then weighted
	// at each pair of on neighbors. Defaults to Potts model (0 if l1==l2, 1 otherwise)
	void setSmoothCost(SmoothCostFn fn);
	void setSmoothCost(SmoothCostFnExtra fn, void *extraData);
	void setSmoothCost(LabelID l1, LabelID l2, EnergyTermType e); 
	void setSmoothCost(EnergyTermType *smoothArray);
	void setSmoothCostFunctor(SmoothCostFunctor* f);
	struct SmoothCostFunctor {
		virtual EnergyTermType compute(SiteID s1, SiteID s2, LabelID l1, LabelID l2) = 0;
	};

	// Sets the cost of using label in the solution. 
	// Set either as uniform cost, or an individual per-label cost.
	void setLabelCost(EnergyTermType cost);
	void setLabelCost(EnergyTermType* costArray);
	void setLabelSubsetCost(LabelID* labels, LabelID numLabels, EnergyTermType cost);

	// Returns current label assigned to input site 
	LabelID whatLabel(SiteID site);
	void    whatLabel(SiteID start, SiteID count, LabelID* labeling);

	// This function can be used to change the label of any site at any time      
	void setLabel(SiteID site, LabelID label);

	// setLabelOrder(false) sets the order to be not random; setLabelOrder(true) 
	//	sets the order to random. By default, the labels are visited in non-random order 
	//	for both the swap and alpha-expansion moves                         
	//	Note that srand() must be initialized with an appropriate seed in order for 
	//	random order to take effect!
	void setLabelOrder(bool isRandom);
	void setLabelOrder(const LabelID* order, LabelID size);

	// Returns total energy for the current labeling
	EnergyType compute_energy();

	// Returns separate Data, Smooth, and Label energy of current labeling 
	EnergyType giveDataEnergy();
	EnergyType giveSmoothEnergy();
	EnergyType giveLabelEnergy();

	// Returns number of sites/labels as specified in the constructor
	SiteID  numSites() const;
	LabelID numLabels() const;

	// Prints output to stdout during exansion/swap execution.
	//   0 => no output
	//   1 => cycle-level output (cycle number, current energy)
	//   2 => expansion-/swap-level output (label(s), current energy)
	void setVerbosity(int level) { m_verbosity = level; }

protected:
	struct LabelCost {
		~LabelCost() { delete [] labels; }
		EnergyTermType cost; 
		bool active;     // flag indicates if this particular labelcost is in effect (i.e. wrt m_labeling)
		VarID aux;
		LabelCost* next; // global list of LabelSetCost records
		LabelID numLabels;
		LabelID* labels;
	};

	struct LabelCostIter {
		LabelCost* node;
		LabelCostIter* next; // local list of LabelSetCost records that contain this label
	};

	LabelID m_num_labels;
	SiteID  m_num_sites;
	LabelID *m_labeling;
	SiteID  *m_lookupSiteVar; // holds index of variable corresponding to site participating in a move,
	                          // -1 for nonparticipating site
	LabelID *m_labelTable;    // to figure out label order in which to do expansion/swaps
	int      m_stepsThisCycle;
	int      m_stepsThisCycleTotal;
	int      m_random_label_order;
	EnergyTermType* m_datacostIndividual;
	EnergyTermType* m_smoothcostIndividual;
	EnergyTermType* m_labelingDataCosts;
	SiteID*         m_labelCounts;
	SiteID*         m_activeLabelCounts;
	LabelCost*      m_labelcostsAll;
	LabelCostIter** m_labelcostsByLabel;
	int             m_labelcostCount;
	bool            m_labelingInfoDirty;
	int             m_verbosity;

	void*   m_datacostFn;
	void*   m_smoothcostFn;
	EnergyType m_beforeExpansionEnergy;

	SiteID *m_numNeighbors;              // holds num of neighbors for each site
	SiteID  m_numNeighborsTotal;         // holds total num of neighbor relationships

	EnergyType (GCoptimization::*m_giveSmoothEnergyInternal)();
	SiteID (GCoptimization::*m_queryActiveSitesExpansion)(LabelID, SiteID*);
	void (GCoptimization::*m_setupDataCostsExpansion)(SiteID,LabelID,EnergyT*,SiteID*);
	void (GCoptimization::*m_setupSmoothCostsExpansion)(SiteID,LabelID,EnergyT*,SiteID*);
	void (GCoptimization::*m_setupDataCostsSwap)(SiteID,LabelID,LabelID,EnergyT*,SiteID*);
	void (GCoptimization::*m_setupSmoothCostsSwap)(SiteID,LabelID,LabelID,EnergyT*,SiteID*);
	void (GCoptimization::*m_applyNewLabeling)(EnergyT*,SiteID*,SiteID,LabelID);
	void (GCoptimization::*m_updateLabelingDataCosts)();

	void (*m_datacostFnDelete)(void* f);
	void (*m_smoothcostFnDelete)(void* f);
	bool (GCoptimization::*m_solveSpecialCases)(EnergyType&);

	// returns a pointer to the neighbors of a site and the weights
	virtual void giveNeighborInfo(SiteID site, SiteID *numSites, SiteID **neighbors, EnergyTermType **weights)=0;
	virtual void finalizeNeighbors() = 0;

	struct DataCostFnFromArray {
		DataCostFnFromArray(EnergyTermType* theArray, LabelID num_labels)
			: m_array(theArray), m_num_labels(num_labels){}
		OLGA_INLINE EnergyTermType compute(SiteID s, LabelID l){return m_array[s*m_num_labels+l];}
	private:
		const EnergyTermType* const m_array;
		const LabelID m_num_labels;
	};

	struct DataCostFnFromFunction {
		DataCostFnFromFunction(DataCostFn fn): m_fn(fn){}
		OLGA_INLINE EnergyTermType compute(SiteID s, LabelID l){return m_fn(s,l);}
	private:
		const DataCostFn m_fn;
	};

	struct DataCostFnFromFunctionExtra {
		DataCostFnFromFunctionExtra(DataCostFnExtra fn,void *extraData): m_fn(fn),m_extraData(extraData){}
		OLGA_INLINE EnergyTermType compute(SiteID s, LabelID l){return m_fn(s,l,m_extraData);}
	private:
		const DataCostFnExtra m_fn;
		void *m_extraData;
	};

	struct SmoothCostFnFromArray {
		SmoothCostFnFromArray(EnergyTermType* theArray, LabelID num_labels)
			: m_array(theArray), m_num_labels(num_labels){}
		OLGA_INLINE EnergyTermType compute(SiteID s1, SiteID s2, LabelID l1, LabelID l2){return m_array[l1*m_num_labels+l2];}
	private:
		const EnergyTermType* const m_array;
		const LabelID m_num_labels;
	};

	struct SmoothCostFnFromFunction {
		SmoothCostFnFromFunction(SmoothCostFn fn)
			: m_fn(fn){}
		OLGA_INLINE EnergyTermType compute(SiteID s1, SiteID s2, LabelID l1, LabelID l2){return m_fn(s1,s2,l1,l2);}
	private:
		const SmoothCostFn m_fn;
	};

	struct SmoothCostFnFromFunctionExtra {
		SmoothCostFnFromFunctionExtra(SmoothCostFnExtra fn,void *extraData)
			: m_fn(fn),m_extraData(extraData){}
		OLGA_INLINE EnergyTermType compute(SiteID s1, SiteID s2, LabelID l1, LabelID l2){return m_fn(s1,s2,l1,l2,m_extraData);}
	private:
		const SmoothCostFnExtra m_fn;
		void *m_extraData;
	};

	struct SmoothCostFnPotts {
		OLGA_INLINE EnergyTermType compute(SiteID, SiteID, LabelID l1, LabelID l2){return l1 != l2 ? (EnergyTermType)1 : (EnergyTermType)0;}
	};

	/////////////////////////////////////////////////////////////////////
	// DataCostFnSparse
	//   This data cost functor maintains a simple sparse structure
	//   to quickly find the cost associated with any (site,label) pair.
	/////////////////////////////////////////////////////////////////////
	class DataCostFnSparse {
		// cLogSitesPerBucket basically controls the compression ratio
		// of the sparse structure: 1 => a dense array, num_sites => a single sparse list.
		// The amount (cLogSitesPerBucket - cLinearSearchSize) determines the maximum 
		// number of binary search steps taken for a cost lookup for specific (site,label).
		//
		static const int cLogSitesPerBucket = 9;
		static const int cSitesPerBucket = (1 << cLogSitesPerBucket); 
		static const size_t    cDataCostPtrMask = ~(sizeof(SparseDataCost)-1);
		static const ptrdiff_t cLinearSearchSize = 64/sizeof(SparseDataCost);

		struct DataCostBucket {
			const SparseDataCost* begin;
			const SparseDataCost* end;     // one-past-the-last item in the range
			const SparseDataCost* predict; // predicts the next cost to be needed
		};

	public:
		DataCostFnSparse(SiteID num_sites, LabelID num_labels);
		DataCostFnSparse(const DataCostFnSparse& src);
		~DataCostFnSparse();

		void           set(LabelID l, const SparseDataCost* costs, SiteID count);
		EnergyTermType compute(SiteID s, LabelID l);
		SiteID         queryActiveSitesExpansion(LabelID alpha_label, const LabelID* labeling, SiteID* activeSites);

		class iterator {
		public:
			OLGA_INLINE iterator(): m_ptr(0) { }
			OLGA_INLINE iterator& operator++() { m_ptr++; return *this; }
			OLGA_INLINE SiteID         site() const { return m_ptr->site; }
			OLGA_INLINE EnergyTermType cost() const { return m_ptr->cost; }
			OLGA_INLINE bool      operator==(const iterator& b) const { return m_ptr == b.m_ptr; }
			OLGA_INLINE bool      operator!=(const iterator& b) const { return m_ptr != b.m_ptr; }
			OLGA_INLINE ptrdiff_t operator- (const iterator& b) const { return m_ptr  - b.m_ptr; }
		private:
			OLGA_INLINE iterator(const SparseDataCost* ptr): m_ptr(ptr) { }
			const SparseDataCost* m_ptr;
			friend class DataCostFnSparse;
		};

		OLGA_INLINE iterator begin(LabelID label) const { return m_buckets[label*m_buckets_per_label].begin; }
		OLGA_INLINE iterator end(LabelID label)   const { return m_buckets[label*m_buckets_per_label + m_buckets_per_label-1].end; }

	private:
		EnergyTermType search(DataCostBucket& b, SiteID s);
		const SiteID  m_num_sites;
		const LabelID m_num_labels;
		const int m_buckets_per_label;
		mutable DataCostBucket* m_buckets;
	};

	template <typename DataCostT> SiteID queryActiveSitesExpansion(LabelID alpha_label, SiteID* activeSites);
	template <typename DataCostT>   void setupDataCostsExpansion(SiteID size,LabelID alpha_label,EnergyT *e,SiteID *activeSites);
	template <typename DataCostT>   void setupDataCostsSwap(SiteID size,LabelID alpha_label,LabelID beta_label,EnergyT *e,SiteID *activeSites);
	template <typename SmoothCostT> void setupSmoothCostsExpansion(SiteID size,LabelID alpha_label,EnergyT *e,SiteID *activeSites);
	template <typename SmoothCostT> void setupSmoothCostsSwap(SiteID size,LabelID alpha_label,LabelID beta_label,EnergyT *e,SiteID *activeSites);
	template <typename DataCostT>   void applyNewLabeling(EnergyT *e,SiteID *activeSites,SiteID size,LabelID alpha_label);
	template <typename DataCostT>   void updateLabelingDataCosts();
	template <typename UserFunctor> void specializeDataCostFunctor(const UserFunctor f);
	template <typename UserFunctor> void specializeSmoothCostFunctor(const UserFunctor f);

	EnergyType setupLabelCostsExpansion(SiteID size,LabelID alpha_label,EnergyT *e,SiteID *activeSites);
	void       updateLabelingInfo(bool updateCounts=true,bool updateActive=true,bool updateCosts=true);
	
	// Check for overflow and submodularity issues when setting up binary graph cut
	void addterm1_checked(EnergyT *e,VarID i,EnergyTermType e0,EnergyTermType e1);
	void addterm1_checked(EnergyT *e,VarID i,EnergyTermType e0,EnergyTermType e1,EnergyTermType w);
	void addterm2_checked(EnergyT *e,VarID i,VarID j,EnergyTermType e00,EnergyTermType e01,EnergyTermType e10,EnergyTermType e11,EnergyTermType w);

	// Returns Smooth Energy of current labeling
	template <typename SmoothCostT> EnergyType giveSmoothEnergyInternal();
	template <typename Functor> static void deleteFunctor(void* f) { delete reinterpret_cast<Functor*>(f); }

	static void handleError(const char *message);
	static void checkInterrupt();

private:
	// Peforms one iteration (one pass over all pairs of labels) of expansion/swap algorithm
	EnergyType oneExpansionIteration();
	EnergyType oneSwapIteration();
	void printStatus1(const char* extraMsg=0);
	void printStatus1(int cycle, bool isSwap, gcoclock_t ticks0);
	void printStatus2(int alpha, int beta, int numVars, gcoclock_t ticks0);
	
	void permuteLabelTable();

	template <typename DataCostT> bool       solveSpecialCases(EnergyType& energy);
	template <typename DataCostT> EnergyType solveGreedy();

	/////////////////////////////////////////////////////////////////////
	// GreedyIter
	//   Lets solveGreedy efficiently traverse the datacosts when 
	//   searching for the next greedy move.
	/////////////////////////////////////////////////////////////////////
	template <typename DataCostT>
	class GreedyIter {
	public:
		GreedyIter(DataCostT& dc, SiteID numSites)
		: m_dc(dc), m_site(0), m_numSites(numSites), m_label(0), m_lbegin(0), m_lend(0)
		{ }

		OLGA_INLINE void start(const LabelID* labels, LabelID labelCount=1)
		{
			m_site = labelCount ? 0 : m_numSites;
			m_label = m_lbegin = labels;
			m_lend = labels + labelCount;
		}
		OLGA_INLINE SiteID site()  const { return m_site; }
		OLGA_INLINE SiteID label() const { return *m_label; }
		OLGA_INLINE bool   done()  const { return m_site == m_numSites; }
		OLGA_INLINE GreedyIter& operator++() 
		{
			// The inner loop is over labels, not sites, to improve memory locality.
			// When dc() is pulling datacosts from an array (the typical format), this can
			// improve performance by a factor of 2x, often more like 4x.
			if (++m_label >= m_lend) {
				m_label = m_lbegin;
				++m_site;
			}
			return *this;
		}
		OLGA_INLINE EnergyTermType compute() const { return m_dc.compute(m_site,*m_label); }
		OLGA_INLINE SiteID feasibleSites() const { return m_numSites; }

	private:
		SiteID m_site;
		DataCostT& m_dc;
		const SiteID m_numSites;
		const LabelID* m_label;
		const LabelID* m_lbegin;
		const LabelID* m_lend;
	};
};


//////////////////////////////////////////////////////////////////////////////////////////////////
// Use this derived class for grid graphs
//////////////////////////////////////////////////////////////////////////////////////////////////

class GCoptimizationGridGraph: public GCoptimization
{
public:
	GCoptimizationGridGraph(SiteID width,SiteID height,LabelID num_labels);
	virtual ~GCoptimizationGridGraph();

	void setSmoothCostVH(EnergyTermType *smoothArray, EnergyTermType *vCosts, EnergyTermType *hCosts);

protected:
	virtual void giveNeighborInfo(SiteID site, SiteID *numSites, SiteID **neighbors, EnergyTermType **weights);
	virtual void finalizeNeighbors();
	EnergyTermType m_unityWeights[4];
	int m_weightedGraph;  // true if spatially varying w_pq's are present. False otherwise.

private:
	SiteID m_width;
	SiteID m_height;
	SiteID *m_neighbors;                 // holds neighbor indexes
	EnergyTermType *m_neighborsWeights;    // holds neighbor weights
	
	void setupNeighbData(SiteID startY,SiteID endY,SiteID startX,SiteID endX,SiteID maxInd,SiteID *indexes);
	void computeNeighborWeights(EnergyTermType *vCosts,EnergyTermType *hCosts);
};

//////////////////////////////////////////////////////////////////////////////////////////////////

class GCoptimizationGeneralGraph:public GCoptimization
{
public:
	// This is the constructor for non-grid graphs. Neighborhood structure must  be specified by 
	// setNeighbors()  function
	GCoptimizationGeneralGraph(SiteID num_sites,LabelID num_labels);
	virtual ~GCoptimizationGeneralGraph();

	// Makes site1 and site2 neighbors of each other. Can be called only 1 time for each      
	// unordered pair of sites. Parameter weight can be used to set spacially varying terms     
	// If the desired penalty for neighboring sites site1 and site2 is                        
	// V(label1,label2) = weight*SmoothnessPenalty(label1,label2), then                        
	// member function setLabel should be called as: setLabel(site1,site2,weight)             
	void setNeighbors(SiteID site1, SiteID site2, EnergyTermType weight=1);

	// passes pointers to arrays storing neighbor information
	// numNeighbors[i] is the number of neighbors for site i
	// neighborsIndexes[i] is a pointer to the array storing the sites which are neighbors to site i
	// neighborWeights[i] is a pointer to array storing the weights between site i and its neighbors
	// in the same order as neighborIndexes[i] stores the indexes
	void setAllNeighbors(SiteID *numNeighbors,SiteID **neighborsIndexes,EnergyTermType **neighborsWeights);

protected: 
	virtual void giveNeighborInfo(SiteID site, SiteID *numSites, SiteID **neighbors, EnergyTermType **weights);
	virtual void finalizeNeighbors();

private:

	typedef struct NeighborStruct{
		SiteID  to_node;
		EnergyTermType weight;
	} Neighbor;

	LinkedBlockList *m_neighbors;
	bool m_needToFinishSettingNeighbors;
	SiteID **m_neighborsIndexes;
	EnergyTermType **m_neighborsWeights;
	bool m_needTodeleteNeighbors;
};


////////////////////////////////////////////////////////////////////
// Methods
////////////////////////////////////////////////////////////////////


OLGA_INLINE GCoptimization::SiteID GCoptimization::numSites() const
{
	return m_num_sites;
}

OLGA_INLINE GCoptimization::LabelID GCoptimization::numLabels() const
{
	return m_num_labels;
}

OLGA_INLINE void GCoptimization::setLabel(SiteID site, LabelID label)
{
	assert(label >= 0 && label < m_num_labels && site >= 0 && site < m_num_sites);
	m_labeling[site] = label;
	m_labelingInfoDirty = true;
}

OLGA_INLINE GCoptimization::LabelID GCoptimization::whatLabel(SiteID site)
{
	assert(site >= 0 && site < m_num_sites);
	return m_labeling[site];
}

#endif
