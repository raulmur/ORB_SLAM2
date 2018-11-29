#ifdef MATLAB_MEX_FILE
#include <mex.h>
#endif
#include <gco-v3.0/GCoptimization.h>
#include <gco-v3.0/LinkedBlockList.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>

// will leave this one just for the laughs :)
//#define olga_assert(expr) assert(!(expr))

// Choose reasonably high-precision timer (sub-millisec resolution if possible).
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#define NOMINMAX
#include <windows.h>

extern "C" gcoclock_t GCO_CLOCKS_PER_SEC = 0;

extern "C" inline gcoclock_t gcoclock() // TODO: not thread safe; separate begin/end so that end doesn't have to check for query frequency
{
	gcoclock_t result = 0;
	if (GCO_CLOCKS_PER_SEC == 0)
		QueryPerformanceFrequency((LARGE_INTEGER*)&GCO_CLOCKS_PER_SEC);
	QueryPerformanceCounter((LARGE_INTEGER*)&result);
	return result;
}

#else
extern "C" {
gcoclock_t GCO_CLOCKS_PER_SEC = CLOCKS_PER_SEC;
}
extern "C" gcoclock_t gcoclock() { return clock(); }
#endif

#ifdef MATLAB_MEX_FILE
extern "C" bool utIsInterruptPending();
static void flushnow()
{
	// Don't flush to frequently, for overall speed.
	static gcoclock_t prevclock = 0;
	gcoclock_t now = gcoclock();
	if (now - prevclock > GCO_CLOCKS_PER_SEC/5) {
		prevclock = now;
		mexEvalString("drawnow;");
	}
}
#define INDEX0 1  // print 1-based label and site indices for MATLAB
#else
inline static bool utIsInterruptPending() { return false; }
static void flushnow() { }
#define INDEX0 0  // print 0-based label and site indices
#endif

// Singly-linked list helper functions; works on any struct with a 'next' member.
template <typename T>
void slist_clear(T*& head)
{
	while (head) {
		T* temp = head;
		head = head->next;
		delete temp;
	}
}

template <typename T>
void slist_prepend(T*& head, T* val)
{
	val->next = head;
	head = val;
}


void GCException::Report() 
{
	printf("\n%s\n",message);
	exit(0);
}



/////////////////////////////////////////////////////////////////////////////////////////////////
//   First we have functions for the base class
/////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor for base class                                                       
GCoptimization::GCoptimization(SiteID nSites, LabelID nLabels) 
: m_num_labels(nLabels)
, m_num_sites(nSites)
, m_datacostIndividual(0)
, m_smoothcostIndividual(0)
, m_labelcostsAll(0)
, m_labelcostsByLabel(0)
, m_labelcostCount(0)
, m_smoothcostFn(0)
, m_datacostFn(0)
, m_numNeighborsTotal(0)
, m_queryActiveSitesExpansion(&GCoptimization::queryActiveSitesExpansion<DataCostFnFromArray>)
, m_setupDataCostsSwap(0)
, m_setupDataCostsExpansion(0)
, m_setupSmoothCostsSwap(0)
, m_setupSmoothCostsExpansion(0)
, m_applyNewLabeling(0)
, m_updateLabelingDataCosts(0)
, m_giveSmoothEnergyInternal(0)
, m_solveSpecialCases(&GCoptimization::solveSpecialCases<DataCostFnFromArray>)
, m_datacostFnDelete(0)
, m_smoothcostFnDelete(0)
, m_random_label_order(false)
, m_verbosity(0)
, m_labelingInfoDirty(true)
, m_lookupSiteVar(new SiteID[nSites])
, m_labeling(new LabelID[nSites])
, m_labelTable(new LabelID[nLabels])
, m_labelingDataCosts(new EnergyTermType[nSites])
, m_labelCounts(new SiteID[nLabels])
, m_activeLabelCounts(new SiteID[m_num_labels])
, m_stepsThisCycle(0)
, m_stepsThisCycleTotal(0)
{
	if ( nLabels <= 1 ) handleError("Number of labels must be >= 2");
	if ( nSites <= 0 )  handleError("Number of sites must be >= 1");
	
	if ( !m_lookupSiteVar || !m_labelTable || !m_labeling ){
		if (m_lookupSiteVar) delete [] m_lookupSiteVar;
		if (m_labelTable) delete [] m_labelTable;
		if (m_labeling) delete [] m_labeling;
		if (m_labelingDataCosts) delete [] m_labelingDataCosts;
		if (m_labelCounts) delete [] m_labelCounts;
		handleError("Not enough memory.");
	}
	
	memset(m_labeling, 0, m_num_sites*sizeof(LabelID));
	memset(m_lookupSiteVar,-1,m_num_sites*sizeof(SiteID));
	setLabelOrder(false);
	specializeSmoothCostFunctor(SmoothCostFnPotts());
}

//-------------------------------------------------------------------

GCoptimization::~GCoptimization()
{
	delete [] m_labelTable;
	delete [] m_lookupSiteVar;
	delete [] m_labeling;
	delete [] m_labelingDataCosts;
	delete [] m_labelCounts;
	delete [] m_activeLabelCounts;

	if (m_datacostFnDelete) m_datacostFnDelete(m_datacostFn);
	if (m_smoothcostFnDelete) m_smoothcostFnDelete(m_smoothcostFn);

	if (m_datacostIndividual) delete [] m_datacostIndividual;
	if (m_smoothcostIndividual) delete [] m_smoothcostIndividual;

	// Delete label cost bookkeeping structures
	//
	slist_clear(m_labelcostsAll);
	if (m_labelcostsByLabel) {
		for ( LabelID i = 0; i < m_num_labels; ++i )
			slist_clear(m_labelcostsByLabel[i]);
		delete [] m_labelcostsByLabel;
	}
}

//-------------------------------------------------------------------

template <>
GCoptimization::SiteID GCoptimization::queryActiveSitesExpansion<GCoptimization::DataCostFnSparse>(LabelID alpha_label,SiteID *activeSites)
{
	return ((DataCostFnSparse*)m_datacostFn)->queryActiveSitesExpansion(alpha_label,m_labeling,activeSites);
}

//-------------------------------------------------------------------

template <>
void GCoptimization::setupDataCostsExpansion<GCoptimization::DataCostFnSparse>(SiteID size,LabelID alpha_label,EnergyT *e,SiteID *activeSites)
{
	DataCostFnSparse* dc = (DataCostFnSparse*)m_datacostFn;
	DataCostFnSparse::iterator dciter = dc->begin(alpha_label);
	for ( SiteID i = 0; i < size; ++i )
	{
		SiteID site = activeSites[i];
		while ( dciter.site() != site )
			++dciter;
		addterm1_checked(e,i,dciter.cost(),m_labelingDataCosts[site]);
	}
}

//-------------------------------------------------------------------

template <>
void GCoptimization::applyNewLabeling<GCoptimization::DataCostFnSparse>(EnergyT *e,SiteID *activeSites,SiteID size,LabelID alpha_label)
{
	DataCostFnSparse* dc = (DataCostFnSparse*)m_datacostFn;
	DataCostFnSparse::iterator dciter = dc->begin(alpha_label);
	for ( SiteID i = 0; i < size; i++ )
	{
		if ( e->get_var(i) == 0 )
		{
			SiteID site = activeSites[i];
			LabelID prev = m_labeling[site];
			m_labeling[site] = alpha_label;
			m_labelCounts[alpha_label]++;
			m_labelCounts[prev]--;
			while ( dciter.site() != site )
				++dciter;
			m_labelingDataCosts[site] = dciter.cost();
		}
	}
	m_labelingInfoDirty = true;
	updateLabelingInfo(false,true,false); // labels have changed, so update necessary labeling info
}

//-------------------------------------------------------------------

template <typename UserFunctor>
void GCoptimization::specializeDataCostFunctor(const UserFunctor f) {
	if ( m_datacostFnDelete )
		m_datacostFnDelete(m_datacostFn);
	if ( m_datacostIndividual )
	{
		delete [] m_datacostIndividual;
		m_datacostIndividual = 0;
	}
	m_datacostFn = new UserFunctor(f);
	m_datacostFnDelete          = &GCoptimization::deleteFunctor<UserFunctor>;
	m_queryActiveSitesExpansion = &GCoptimization::queryActiveSitesExpansion<UserFunctor>;
	m_setupDataCostsExpansion   = &GCoptimization::setupDataCostsExpansion<UserFunctor>;
	m_setupDataCostsSwap        = &GCoptimization::setupDataCostsSwap<UserFunctor>;
	m_applyNewLabeling          = &GCoptimization::applyNewLabeling<UserFunctor>;
	m_updateLabelingDataCosts   = &GCoptimization::updateLabelingDataCosts<UserFunctor>;
	m_solveSpecialCases         = &GCoptimization::solveSpecialCases<UserFunctor>;
}

template <typename UserFunctor>
void GCoptimization::specializeSmoothCostFunctor(const UserFunctor f) {
	if ( m_smoothcostFnDelete )
		m_smoothcostFnDelete(m_smoothcostFn);
	if ( m_smoothcostIndividual )
	{
		delete [] m_smoothcostIndividual;
		m_smoothcostIndividual = 0;
	}
	m_smoothcostFn = new UserFunctor(f);
	m_smoothcostFnDelete        = &GCoptimization::deleteFunctor<UserFunctor>;
	m_giveSmoothEnergyInternal  = &GCoptimization::giveSmoothEnergyInternal<UserFunctor>;
	m_setupSmoothCostsExpansion = &GCoptimization::setupSmoothCostsExpansion<UserFunctor>;
	m_setupSmoothCostsSwap      = &GCoptimization::setupSmoothCostsSwap<UserFunctor>;
}

//-------------------------------------------------------------------

template <typename SmoothCostT>
GCoptimization::EnergyType GCoptimization::giveSmoothEnergyInternal()
{
	EnergyType eng = (EnergyType) 0;
	SiteID i,numN,*nPointer,nSite,n;
	EnergyTermType *weights;
	SmoothCostT* sc = (SmoothCostT*) m_smoothcostFn;
	for ( i = 0; i < m_num_sites; i++ )
	{
		giveNeighborInfo(i,&numN,&nPointer,&weights);
		for ( n = 0; n < numN; n++ )
		{
			nSite = nPointer[n];
			if ( nSite < i ) 
				eng += weights[n]*(sc->compute(i,nSite,m_labeling[i],m_labeling[nSite]));
		}
	}

	return eng;
}

//-------------------------------------------------------------------

OLGA_INLINE void GCoptimization::addterm1_checked(EnergyT* e, VarID i, EnergyTermType e0, EnergyTermType e1)
{
	if ( e0 > GCO_MAX_ENERGYTERM || e1 > GCO_MAX_ENERGYTERM )
		handleError("Data cost term was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
	m_beforeExpansionEnergy += e1;
	e->add_term1(i,e0,e1);
}

OLGA_INLINE void GCoptimization::addterm1_checked(EnergyT* e, VarID i, EnergyTermType e0, EnergyTermType e1, EnergyTermType w)
{
	if ( e0 > GCO_MAX_ENERGYTERM || e1 > GCO_MAX_ENERGYTERM )
		handleError("Smooth cost term was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
	if ( w > GCO_MAX_ENERGYTERM )
		handleError("Smoothness weight was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
	m_beforeExpansionEnergy += e1*w;
	e->add_term1(i,e0*w,e1*w);
}

OLGA_INLINE void GCoptimization::addterm2_checked(EnergyT* e, VarID i, VarID j, EnergyTermType e00, EnergyTermType e01, EnergyTermType e10, EnergyTermType e11, EnergyTermType w)
{
	if ( e00 > GCO_MAX_ENERGYTERM || e11 > GCO_MAX_ENERGYTERM || e01 > GCO_MAX_ENERGYTERM || e10 > GCO_MAX_ENERGYTERM )
		handleError("Smooth cost term was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
	if ( w > GCO_MAX_ENERGYTERM )
		handleError("Smoothness weight was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
	// Inside energy/maxflow code the submodularity check is performed as an assertion,
	// but is optimized out. We check it in release builds as well.
	if ( e00+e11 > e01+e10 )
		handleError("Non-submodular expansion term detected; smooth costs must be a metric for expansion");
	m_beforeExpansionEnergy += e11*w;
	e->add_term2(i,j,e00*w,e01*w,e10*w,e11*w);
}

//------------------------------------------------------------------

template <typename DataCostT>
GCoptimization::SiteID GCoptimization::queryActiveSitesExpansion(LabelID alpha_label,SiteID *activeSites)
{
	SiteID size = 0;
	for ( SiteID i = 0; i < m_num_sites; i++ )
		if ( m_labeling[i] != alpha_label )
			activeSites[size++] = i;
	return size;
}

//-------------------------------------------------------------------

template <typename DataCostT>
void GCoptimization::setupDataCostsExpansion(SiteID size,LabelID alpha_label,EnergyT *e,SiteID *activeSites)
{
	DataCostT* dc = (DataCostT*)m_datacostFn;
	for ( SiteID i = 0; i < size; ++i )
		addterm1_checked(e,i,dc->compute(activeSites[i],alpha_label),m_labelingDataCosts[activeSites[i]]);
}

//-------------------------------------------------------------------

template <typename SmoothCostT>
void GCoptimization::setupSmoothCostsExpansion(SiteID size,LabelID alpha_label,EnergyT *e,SiteID *activeSites)
{
	SiteID i,nSite,site,n,nNum,*nPointer;
	EnergyTermType *weights;
	SmoothCostT* sc = (SmoothCostT*)m_smoothcostFn;

	for ( i = size - 1; i >= 0; i-- )
	{
		site = activeSites[i];
		giveNeighborInfo(site,&nNum,&nPointer,&weights);
		for ( n = 0; n < nNum; n++ )
		{
			nSite = nPointer[n];
			if ( m_lookupSiteVar[nSite] == -1 ) 
				addterm1_checked(e,i,sc->compute(site,nSite,alpha_label,m_labeling[nSite]),
				                     sc->compute(site,nSite,m_labeling[site],m_labeling[nSite]),weights[n]);
			else if ( nSite < site ) 
			{
				addterm2_checked(e,i,m_lookupSiteVar[nSite],
				                 sc->compute(site,nSite,alpha_label,alpha_label),
				                 sc->compute(site,nSite,alpha_label,m_labeling[nSite]),
				                 sc->compute(site,nSite,m_labeling[site],alpha_label),
				                 sc->compute(site,nSite,m_labeling[site],m_labeling[nSite]),weights[n]);
			}
		}
	}
}

//-----------------------------------------------------------------------------------

template <typename DataCostT>
void GCoptimization::setupDataCostsSwap(SiteID size, LabelID alpha_label, LabelID beta_label,
										 EnergyT *e,SiteID *activeSites )
{
	DataCostT* dc = (DataCostT*)m_datacostFn;
	for ( SiteID i = 0; i < size; i++ )
	{
		e->add_term1(i,dc->compute(activeSites[i],alpha_label),
		               dc->compute(activeSites[i],beta_label) );
	}
}

//-------------------------------------------------------------------

template <typename SmoothCostT>
void GCoptimization::setupSmoothCostsSwap(SiteID size, LabelID alpha_label,LabelID beta_label,
										 EnergyT *e,SiteID *activeSites )
{
	SiteID i,nSite,site,n,nNum,*nPointer;
	EnergyTermType *weights;
	SmoothCostT* sc = (SmoothCostT*)m_smoothcostFn;

	for ( i = size - 1; i >= 0; i-- )
	{
		site = activeSites[i];
		giveNeighborInfo(site,&nNum,&nPointer,&weights);
		for ( n = 0; n < nNum; n++ )
		{
			nSite = nPointer[n];
			if ( m_lookupSiteVar[nSite] == -1 )
				addterm1_checked(e,i,sc->compute(site,nSite,alpha_label,m_labeling[nSite]),
				                     sc->compute(site,nSite,beta_label, m_labeling[nSite]),weights[n]);
			else if ( nSite < site )
			{
				addterm2_checked(e,i,m_lookupSiteVar[nSite],
				                 sc->compute(site,nSite,alpha_label,alpha_label),
				                 sc->compute(site,nSite,alpha_label,beta_label),
				                 sc->compute(site,nSite,beta_label,alpha_label),
				                 sc->compute(site,nSite,beta_label,beta_label),weights[n]);
			}
		}
	}
}

//-----------------------------------------------------------------------------------

template <typename DataCostT>
void GCoptimization::applyNewLabeling(EnergyT *e,SiteID *activeSites,SiteID size,LabelID alpha_label)
{
	DataCostT* dc = (DataCostT*)m_datacostFn;
	for ( SiteID i = 0; i < size; i++ )
	{
		if ( e->get_var(i) == 0 )
		{
			SiteID site = activeSites[i];
			LabelID prev = m_labeling[site];
			m_labeling[site] = alpha_label;
			m_labelCounts[alpha_label]++;
			m_labelCounts[prev]--;
			m_labelingDataCosts[site] = dc->compute(site,alpha_label);
		}
	}
	m_labelingInfoDirty = true;
	updateLabelingInfo(false,true,false); // labels have changed, so update necessary labeling info
}

//-----------------------------------------------------------------------------------

template <typename DataCostT>
void GCoptimization::updateLabelingDataCosts()
{
	DataCostT* dc = (DataCostT*)m_datacostFn;
	for (int i = 0; i < m_num_sites; ++i)
		m_labelingDataCosts[i] = dc->compute(i,m_labeling[i]);
}

//-----------------------------------------------------------------------------------

template <typename DataCostT>
bool GCoptimization::solveSpecialCases(EnergyType& energy)
{
	finalizeNeighbors();

	DataCostT* dc = (DataCostT*)m_datacostFn;
	bool sc = m_numNeighborsTotal != 0;
	bool lc = m_labelcostsAll != 0;

	if ( !dc && !sc && !lc )
	{
		energy = 0;
		return true;
	}

	if ( dc && !sc && !lc ) {
		// Special case: No label costs, so return trivial solution
		energy = 0;
		for ( SiteID i = 0; i < m_num_sites; ++i ) {
			LabelID minCostLabel = 0;
			EnergyTermType minCost = dc->compute(i, 0);
			for ( LabelID l = 1; l < m_num_labels; ++l ) {
				EnergyTermType lcost = dc->compute(i, l);
				if ( lcost < minCost ) {
					minCostLabel = l;
					minCost = lcost;
				}
			}
			if ( minCostLabel > GCO_MAX_ENERGYTERM )
				handleError("Data cost was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
			m_labeling[i] = minCostLabel;
			energy += minCost;
		}
		m_labelingInfoDirty = true;
		updateLabelingInfo();
		return true;
	}

	if ( !dc && !sc && lc ) {
		// Special case: No data costs, so return trivial solution
		LabelID minLabel = 0;
		EnergyType minLabelCost = GCO_MAX_ENERGYTERM*(EnergyType)m_num_labels;
		for ( LabelID l = 0; l < m_num_labels; ++l ) {
			EnergyType lcsum = 0;
			for ( LabelCostIter* lci = m_labelcostsByLabel[l]; lci; lci = lci->next )
				lcsum += lci->node->cost;
			if ( lcsum < minLabelCost ) {
				minLabel = l;
				minLabelCost = lcsum;
			}
		}
		for ( SiteID i = 0; i < m_num_sites; ++i )
			m_labeling[i] = minLabel;
		energy = minLabelCost;
		m_labelingInfoDirty = true;
		updateLabelingInfo();
		return true;
	}

	if ( dc && !sc && lc ) {
		LabelCost* lc;
		for ( lc = m_labelcostsAll; lc; lc = lc->next )
			if ( lc->numLabels > 1)
				break;
		if ( !lc ) {
			// Special case: Data costs and per-label costs 
			energy = solveGreedy<DataCostT>();
			return true;
		}
	}

	// Otherwise, use full-blown expansion/swap
	return false;
}

template <>
class GCoptimization::GreedyIter<GCoptimization::DataCostFnSparse> {
public:
	GreedyIter(DataCostFnSparse& dc, SiteID)
	: m_dc(dc), m_label(0), m_labelend(0)
	{ }

	OLGA_INLINE void start(const LabelID* labels, LabelID labelCount=1)
	{
		m_label = labels;
		m_labelend = labels + labelCount;
		if (labelCount > 0) {
			m_site = m_dc.begin(*labels);
			m_siteend = m_dc.end(*labels);
			while (m_site == m_siteend) {
				if (++m_label == m_labelend)
					break;
				m_site     = m_dc.begin(*m_label);
				m_siteend  = m_dc.end(*m_label);
			}
		}
	}
	OLGA_INLINE SiteID site()  const { return m_site.site(); }
	OLGA_INLINE SiteID label() const { return *m_label; }
	OLGA_INLINE bool   done()  const { return m_label >= m_labelend; }
	OLGA_INLINE GreedyIter& operator++() 
	{
		// The inner loop is over sites, not labels, because sparse data costs 
		// are stored as consecutive [sparse] SiteIDs with respect to each label.
		if (++m_site == m_siteend) {
			while (++m_label < m_labelend) {
				m_site     = m_dc.begin(*m_label);
				m_siteend  = m_dc.end(*m_label);
				if (m_site != m_siteend)
					break;
			}
		}
		return *this;
	}
	OLGA_INLINE EnergyTermType compute() const { return m_site.cost(); }
	OLGA_INLINE SiteID feasibleSites() const { return (SiteID)(m_siteend - m_site); }

private:
	DataCostFnSparse::iterator m_site;
	DataCostFnSparse::iterator m_siteend;
	DataCostFnSparse& m_dc;
	const LabelID* m_label;
	const LabelID* m_labelend;
};

template <typename DataCostT>
GCoptimization::EnergyType GCoptimization::solveGreedy()
{
	printStatus1("starting greedy algorithm (1 cycle only)");
	m_stepsThisCycle = m_stepsThisCycleTotal = 0;

	EnergyType estart = compute_energy();
	EnergyType efinal = 0;
	LabelID* oldLabeling = m_labeling;
	m_labeling = new LabelID[m_num_sites];
	EnergyType* e = new EnergyType[m_num_labels];
	LabelID* order = new LabelID[m_num_labels];  // order[0..activeCount-1] contains the activated labels so far

	try {
		gcoclock_t ticks0all = gcoclock();
		gcoclock_t ticks0 = gcoclock();

		// clear active flags
		for ( LabelCost* lc = m_labelcostsAll; lc; lc = lc->next)
			lc->active = false;

		DataCostT* dc = (DataCostT*)m_datacostFn;
		GreedyIter<DataCostT> iter(*dc,m_num_sites);
		LabelID alpha = 0;

		// Treat first iteration as special case. 
		// Ignore current labeling and just find the greedy initial label.
		for ( LabelID l = 0; l < m_num_labels; ++l ) {
			e[l] = 0;
			for ( LabelCostIter* lci = m_labelcostsByLabel[l]; lci; lci = lci->next )
				e[l] += lci->node->cost;
			iter.start(&l);
			e[l] += (EnergyType)(m_num_sites - iter.feasibleSites()) * GCO_MAX_ENERGYTERM; // pre-add GCO_MAX_ENERGYTERM for all infeasible sites
			for (; !iter.done(); ++iter) {
				EnergyTermType dataCost = iter.compute();
				if ( dataCost > GCO_MAX_ENERGYTERM )
					handleError("Data cost was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
				e[l] += dataCost;
				if ( e[l] > e[alpha] ) // break out early if this will definitely 
					break;             // not be a good label to start from
			}
			if ( e[l] < e[alpha] ) // choose alpha with minimum energy e[alpha]
				alpha = l;
		}
		for ( SiteID i = 0; i < m_num_sites; ++i ) {
			m_labeling[i] = alpha;
			m_labelingDataCosts[i] = dc->compute(i,alpha);
		}
		for ( LabelCostIter* lci = m_labelcostsByLabel[alpha]; lci; lci = lci->next )
			lci->node->active = true;

		// List of labels in the order that they were expanded upon (order[0] first, order[1] second, ...)
		for ( LabelID l = 0; l < m_num_labels; ++l )
			order[l] = l;
		order[alpha] = 0;
		order[0] = alpha;

		printStatus2(alpha,-1,m_num_sites,ticks0);

		// Greedily expand remaining labels
		for ( LabelID alpha_count = 1; alpha_count <= m_num_labels; ++alpha_count) {
			checkInterrupt();
			ticks0 = gcoclock();

			// Energy e[l] for expanding on label 'l' starts at e[alpha] + new labelcosts for introducing l
			LabelID alpha_prev = alpha;
			for ( LabelID li = alpha_count; li < m_num_labels; ++li ) {
				LabelID l = order[li];
				e[l] = e[alpha_prev];
				for ( LabelCostIter* lci = m_labelcostsByLabel[l]; lci; lci = lci->next )
					if ( !lci->node->active )
						e[l] += lci->node->cost;
			}

			// Loop over all sites and all remaining labels to calculate energy drop.
			for ( iter.start(&order[alpha_count],m_num_labels-alpha_count); !iter.done(); ++iter ) {
				EnergyTermType dc_l = iter.compute();
				EnergyTermType dc_i = m_labelingDataCosts[iter.site()];
				EnergyTermType delta_i = dc_l - dc_i;
				if ( delta_i < 0 )
					e[iter.label()] += delta_i;
			}

			// Choose the next alpha based on lowest resulting energy
			LabelID alpha_index = alpha_count-1;
			for ( LabelID li = alpha_count; li < m_num_labels; ++li ) {
				LabelID l = order[li];
				if ( e[l] < e[alpha] ) {
					alpha = l;
					alpha_index = li;
				}
			}

			if ( alpha == alpha_prev )
				break;

			// Append alpha to the list of activated labels
			LabelID temp = order[alpha_count];
			order[alpha_count] = order[alpha_index];
			order[alpha_index] = temp;

			// Apply the new labeling, updating m_labelingDataCosts and active labelcosts as necessary
			iter.start(&alpha);
			SiteID size = iter.feasibleSites();
			for ( ; !iter.done(); ++iter ) {
				EnergyTermType dc_l = iter.compute();
				EnergyTermType dc_i = m_labelingDataCosts[iter.site()];
				EnergyTermType delta_i = dc_l - dc_i;
				if ( delta_i < 0 ) {
					m_labeling[iter.site()] = alpha;
					m_labelingDataCosts[iter.site()] = dc_l;
				}
			}
			for ( LabelCostIter* lci = m_labelcostsByLabel[alpha]; lci; lci = lci->next )
				lci->node->active = true;
			printStatus2(alpha,-1,size,ticks0);
		}

		efinal = e[alpha];
		if ( efinal < estart ) {
			// Greedy succeeded in lowering energy compared to initial labeling
			delete [] oldLabeling;
			m_labelingInfoDirty = true;
			updateLabelingInfo(true,false,false); // update m_labelCounts only; m_labelingDataCosts and active labelcosts should be up to date
			printStatus1(1,false,ticks0all);
		} else {
			// Greedy failed to find a lower energy, so revert everything
			efinal = estart;
			delete [] m_labeling;
			m_labeling = oldLabeling;
			m_labelingInfoDirty = true;
			updateLabelingInfo(); // put all labeling info back the way it was
			printStatus1(1,false,ticks0all);
		}

		delete [] order;
		delete [] e;
	} catch (...) {
		delete [] order;
		delete [] e;
		throw;
	}
	return efinal;
}

//------------------------------------------------------------------

void GCoptimization::setDataCost(DataCostFn fn) { 
	specializeDataCostFunctor(DataCostFnFromFunction(fn));
	m_labelingInfoDirty = true;
}


//------------------------------------------------------------------

void GCoptimization::setDataCost(DataCostFnExtra fn, void *extraData) { 
	specializeDataCostFunctor(DataCostFnFromFunctionExtra(fn, extraData));
	m_labelingInfoDirty = true;
}

//-------------------------------------------------------------------

void GCoptimization::setDataCost(EnergyTermType *dataArray) {
	specializeDataCostFunctor(DataCostFnFromArray(dataArray, m_num_labels));
	m_labelingInfoDirty = true;
}

//-------------------------------------------------------------------

void GCoptimization::setDataCost(SiteID s, LabelID l, EnergyTermType e) {
	if ( !m_datacostIndividual )
	{
		EnergyTermType* table = new EnergyTermType[m_num_sites*m_num_labels];
		memset(table, 0, m_num_sites*m_num_labels*sizeof(EnergyTermType));
		specializeDataCostFunctor(DataCostFnFromArray(table, m_num_labels));
		m_datacostIndividual = table;
		m_labelingInfoDirty = true;
	}
	m_datacostIndividual[s*m_num_labels + l] = e;
	if ( m_labeling[s] == l )
		m_labelingInfoDirty = true; // m_labelingDataCosts is dirty
}

//-------------------------------------------------------------------

void GCoptimization::setDataCostFunctor(DataCostFunctor* f) {
	if ( m_datacostFnDelete )
		m_datacostFnDelete(m_datacostFn);
	if ( m_datacostIndividual )
	{
		delete [] m_datacostIndividual;
		m_datacostIndividual = 0;
	}
	m_datacostFn = f;
	m_datacostFnDelete          = 0;
	m_queryActiveSitesExpansion = &GCoptimization::queryActiveSitesExpansion<DataCostFunctor>;
	m_setupDataCostsExpansion   = &GCoptimization::setupDataCostsExpansion<DataCostFunctor>;
	m_setupDataCostsSwap        = &GCoptimization::setupDataCostsSwap<DataCostFunctor>;
	m_applyNewLabeling          = &GCoptimization::applyNewLabeling<DataCostFunctor>;
	m_updateLabelingDataCosts   = &GCoptimization::updateLabelingDataCosts<DataCostFunctor>;
	m_solveSpecialCases         = &GCoptimization::solveSpecialCases<DataCostFunctor>;
	m_labelingInfoDirty = true;
}

//-------------------------------------------------------------------

void GCoptimization::setDataCost(LabelID l, SparseDataCost *costs, SiteID count)
{
	if ( !m_datacostFn )
		specializeDataCostFunctor(DataCostFnSparse(numSites(),numLabels()));
	else if ( m_queryActiveSitesExpansion != (SiteID (GCoptimization::*)(LabelID,SiteID*))&GCoptimization::queryActiveSitesExpansion<DataCostFnSparse> )
		handleError("Cannot apply sparse data costs after dense data costs have been used.");
	m_labelingInfoDirty = true;
	DataCostFnSparse* dc = (DataCostFnSparse*)m_datacostFn;
	dc->set(l,costs,count);
}

//-------------------------------------------------------------------

void GCoptimization::setSmoothCost(SmoothCostFn fn) {
	specializeSmoothCostFunctor(SmoothCostFnFromFunction(fn));
}

//-------------------------------------------------------------------

void GCoptimization::setSmoothCost(SmoothCostFnExtra fn, void* extraData) {
	specializeSmoothCostFunctor(SmoothCostFnFromFunctionExtra(fn, extraData));
}

//-------------------------------------------------------------------

void GCoptimization::setSmoothCost(EnergyTermType *smoothArray) {
	specializeSmoothCostFunctor(SmoothCostFnFromArray(smoothArray, m_num_labels));
}

//-------------------------------------------------------------------

void GCoptimization::setSmoothCost(LabelID l1, LabelID l2, EnergyTermType e){
	if ( !m_smoothcostIndividual )
	{
		EnergyTermType* table = new EnergyTermType[m_num_labels*m_num_labels];
		memset(table, 0, m_num_labels*m_num_labels*sizeof(EnergyTermType));
		specializeSmoothCostFunctor(SmoothCostFnFromArray(table, m_num_labels));
		m_smoothcostIndividual = table;
	} 
	m_smoothcostIndividual[l1*m_num_labels + l2] = e;
}

//-------------------------------------------------------------------

void GCoptimization::setSmoothCostFunctor(SmoothCostFunctor* f) {
	if ( m_smoothcostFnDelete )
		m_smoothcostFnDelete(m_smoothcostFn);
	if ( m_smoothcostIndividual )
	{
		delete [] m_smoothcostIndividual;
		m_smoothcostIndividual = 0;
	}
	m_smoothcostFn = f;
	m_smoothcostFnDelete        = 0;
	m_giveSmoothEnergyInternal  = &GCoptimization::giveSmoothEnergyInternal<SmoothCostFunctor>;
	m_setupSmoothCostsExpansion = &GCoptimization::setupSmoothCostsExpansion<SmoothCostFunctor>;
	m_setupSmoothCostsSwap      = &GCoptimization::setupSmoothCostsSwap<SmoothCostFunctor>;
}

//-------------------------------------------------------------------

void GCoptimization::setLabelCost(EnergyTermType cost) 
{
	EnergyTermType* lc = new EnergyTermType[m_num_labels];
	for ( LabelID i = 0; i < m_num_labels; ++i )
		lc[i] = cost;
	setLabelCost(lc);
	delete [] lc;
}

//-------------------------------------------------------------------

void GCoptimization::setLabelCost(EnergyTermType *costArray) 
{
	for ( LabelID i = 0; i < m_num_labels; ++i )
		setLabelSubsetCost(&i, 1, costArray[i]);
}

//-------------------------------------------------------------------

void GCoptimization::setLabelSubsetCost(LabelID* labels, LabelID numLabels, EnergyTermType cost)
{
	if ( cost < 0 )
		handleError("Label costs must be non-negative.");
	if ( cost > GCO_MAX_ENERGYTERM )
		handleError("Label cost was larger than GCO_MAX_ENERGYTERM; danger of integer overflow.");
	for ( LabelID i = 0; i < numLabels; ++i)
		if ( labels[i] < 0 || labels[i] >= m_num_labels )
			handleError("Invalid label id was found in label subset list.");

	if ( !m_labelcostsByLabel ) {
		m_labelcostsByLabel = new LabelCostIter*[m_num_labels];
		memset(m_labelcostsByLabel, 0, m_num_labels*sizeof(void*));
	}

	// If this particular subset already has a cost, simply replace it.
	for ( LabelCostIter* lci = m_labelcostsByLabel[labels[0]]; lci; lci = lci->next ) {
		if ( numLabels == lci->node->numLabels ) {
			if ( !memcmp(labels, lci->node->labels, numLabels*sizeof(LabelID)) ) {
				// This label subset already exists, so just update the cost and return
				lci->node->cost = cost;
				return;
			}
		}
	}
	
	if (cost == 0)
		return;

	// Create a new LabelCost entry and add it to the appropriate lists
	m_labelcostCount++;
	LabelCost* lc = new LabelCost;
	lc->cost = cost; 
	lc->active = false;
	lc->aux = -1;
	lc->numLabels = numLabels;
	lc->labels = new LabelID[numLabels];
	memcpy(lc->labels, labels, numLabels*sizeof(LabelID));
	slist_prepend(m_labelcostsAll, lc);
	for ( LabelID i = 0; i < numLabels; ++i ) {
		LabelCostIter* lci = new LabelCostIter;
		lci->node = lc; 
		slist_prepend(m_labelcostsByLabel[labels[i]], lci);
	}
}

//-------------------------------------------------------------------

void GCoptimization::whatLabel(SiteID start, SiteID count, LabelID* labeling)
{
	assert(start >= 0 && start+count <= m_num_sites);
	memcpy(labeling, m_labeling+start, count*sizeof(LabelID));
}

//-------------------------------------------------------------------

GCoptimization::EnergyType GCoptimization::giveSmoothEnergy()
{
	finalizeNeighbors();
	if ( m_giveSmoothEnergyInternal ) 
		return( (this->*m_giveSmoothEnergyInternal)());
	return 0;
}

//-------------------------------------------------------------------

GCoptimization::EnergyType GCoptimization::giveDataEnergy()
{
	updateLabelingInfo();
	EnergyType energy = 0;
	for ( SiteID i = 0; i < m_num_sites; i++ )
		energy += m_labelingDataCosts[i];
	return energy;
}

GCoptimization::EnergyType GCoptimization::giveLabelEnergy()
{
	updateLabelingInfo();
	EnergyType energy = 0;
	for ( LabelCost* lc = m_labelcostsAll; lc; lc = lc->next)
		if ( lc->active )
			energy += lc->cost;
	return energy;
}

//-------------------------------------------------------------------
 
GCoptimization::EnergyType GCoptimization::compute_energy()
{
	return giveDataEnergy() + giveSmoothEnergy() + giveLabelEnergy();
}

//-------------------------------------------------------------------

void GCoptimization::permuteLabelTable()
{
	if ( !m_random_label_order )
		return;
	for ( LabelID i = 0; i < m_num_labels; i++ )
	{
		LabelID j = i + (rand() % (m_num_labels-i));
		LabelID temp    = m_labelTable[i];
		m_labelTable[i] = m_labelTable[j];
		m_labelTable[j] = temp;
	}
}

//-------------------------------------------------------------------

GCoptimization::EnergyType GCoptimization::expansion(int max_num_iterations)
{
	EnergyType new_energy, old_energy;
	if ( (this->*m_solveSpecialCases)(new_energy) )
		return new_energy;

	permuteLabelTable();
	updateLabelingInfo();

	try 
	{
		if ( max_num_iterations == -1 )
		{
			// Strategic expansion loop focuses on labels that successfuly reduced the energy
			printStatus1("starting alpha-expansion w/ adaptive cycles");
			std::vector<LabelID> queueSizes;
			queueSizes.push_back(m_num_labels);

			int cycle = 1;
			LabelID next = 0;
			do
			{
				gcoclock_t ticks0 = gcoclock();
				m_stepsThisCycle = 0; 

				// Make a pass over the unchecked labels in the current queue, i.e. m_labelTable[next..queueSize-1]
				LabelID queueSize = queueSizes.back();
				LabelID start = next;
				m_stepsThisCycleTotal = queueSize - start;
				do 
				{
					if ( !alpha_expansion(m_labelTable[next]) )
						std::swap(m_labelTable[next],m_labelTable[--queueSize]); // don't put this label in a new queue
					else
						++next; // keep this label for the next (smaller) queue
					m_stepsThisCycle++;
				} while ( next < queueSize );

				if ( next == start )  // No expansion was successful, so try more labels from the previous queue
				{
					next = queueSizes.back();
					queueSizes.pop_back();
				}
				else if ( queueSize < queueSizes.back()/2 ) // Some expansions were successful, so focus on them in a new queue
				{
					next = 0;
					queueSizes.push_back(queueSize);
				}
				else
					next = 0;  // All expansions were successful, so do another complete sweep
				
				printStatus1(cycle++,false,ticks0);
			} while ( !queueSizes.empty() );
			new_energy = compute_energy();
		}
		else
		{
			// Standard expansion loop sweeps over all labels each cycle
			printStatus1("starting alpha-expansion w/ standard cycles");
			new_energy = compute_energy();
			old_energy = new_energy+1;
			for ( int cycle = 1; cycle <= max_num_iterations; cycle++ )
			{
				gcoclock_t ticks0 = gcoclock();
				old_energy = new_energy;
				new_energy = oneExpansionIteration();
				printStatus1(cycle,false,ticks0);
				if ( new_energy == old_energy )
					break;
				permuteLabelTable();
			}
		}
	} 
	catch (...)
	{
		m_stepsThisCycle = m_stepsThisCycleTotal = 0;
		throw;
	}
	m_stepsThisCycle = m_stepsThisCycleTotal = 0; // set so that alpha_expansion() knows it's no inside expansion() if called externally
	return new_energy;
}

//-------------------------------------------------------------------

void GCoptimization::setLabelOrder(bool isRandom)
{
	m_random_label_order = isRandom;
	for ( LabelID i = 0; i < m_num_labels; i++ )
		m_labelTable[i] = i;
}

//-------------------------------------------------------------------

void GCoptimization::setLabelOrder(const LabelID* order, LabelID size)
{
	if ( size > m_num_labels )
		handleError("setLabelOrder receieved too many labels");
	for ( LabelID i = 0; i < size; ++i )
		if ( order[i] < 0 || order[i] >= m_num_labels )
			handleError("Invalid label id in setLabelOrder");
	m_random_label_order = false;
	memcpy(m_labelTable,order,size*sizeof(LabelID));
	memset(m_labelTable+size,-1,(m_num_labels-size)*sizeof(LabelID));
}

//------------------------------------------------------------------

void GCoptimization::handleError(const char *message)
{
	throw GCException(message);
}

//------------------------------------------------------------------

void GCoptimization::checkInterrupt()
{
	if ( utIsInterruptPending() )
		throw GCException("Interrupted.");
}


//-------------------------------------------------------------------//
//                  METHODS for EXPANSION MOVES                      //  
//-------------------------------------------------------------------//

GCoptimization::EnergyType GCoptimization::setupLabelCostsExpansion(SiteID size,LabelID alpha_label,EnergyT *e,SiteID *activeSites)
{
	EnergyType alphaCostCorrection = 0;
	if ( !m_labelcostsAll )
		return alphaCostCorrection;

	const SiteID DISABLE = -2;
	const SiteID UNINIT  = -1;
	for ( LabelCost* lc = m_labelcostsAll; lc; lc = lc->next )
		lc->aux = UNINIT;

	// Skip higher-order costs that include alpha_label or any label used
	// outside the activeSites, since they cannot be eliminated by the expansion.
	if ( m_queryActiveSitesExpansion == (SiteID (GCoptimization::*)(LabelID,SiteID*))&GCoptimization::queryActiveSitesExpansion<DataCostFnSparse> )
	{
		// For sparse data costs, things are more complicated, because we must ensure that
		// no label cost for a fixed (non-active) non-alpha label is encoded in the graph.
		memset(m_activeLabelCounts,0,m_num_labels*sizeof(SiteID));
		for ( SiteID i = 0; i < size; ++i )
			m_activeLabelCounts[m_labeling[activeSites[i]]]++;

		for ( LabelID l = 0; l < m_num_labels; ++l )
		{
			if ( m_activeLabelCounts[l] != m_labelCounts[l] )
			{
				for ( LabelCostIter* lcj = m_labelcostsByLabel[l]; lcj; lcj = lcj->next )
					lcj->node->aux = DISABLE;
			}
		}
	}
	for ( LabelCostIter* lci = m_labelcostsByLabel[alpha_label]; lci; lci = lci->next )
		lci->node->aux = DISABLE;

	// Since we're explicitly omitting the alpha_label label costs from the binary energy, 
	// calculate what it would have been, so that we can potentially reject the expansion afterwards.
	if ( !m_labelCounts[alpha_label] )
	{
		for ( LabelCostIter* lci = m_labelcostsByLabel[alpha_label]; lci; lci = lci->next )
			if ( !lci->node->active )
				alphaCostCorrection += lci->node->cost;
	}

	// Add edges to the graph, including auxiliary vertices as needed
	for ( SiteID i = 0; i < size; i++ )
	{
		LabelID label_i = m_labeling[activeSites[i]];
		for ( LabelCostIter* lci = m_labelcostsByLabel[label_i]; lci; lci = lci->next ) 
		{
			LabelCost* lc = lci->node;
			if ( lc->aux == DISABLE )
				continue;

			// Add auxiliary variable if necessary, and add pairwise potential
			if ( lc->aux == UNINIT ) 
			{
				lc->aux = e->add_variable();
				e->add_term1(lc->aux,0,lc->cost);
				m_beforeExpansionEnergy += lc->cost;
			}
			e->add_term2(i,lc->aux,0,0,lc->cost,0);
		}
	}

	return alphaCostCorrection;
}

//-------------------------------------------------------------------
void GCoptimization::updateLabelingInfo(bool updateCounts, bool updateActive, bool updateCosts)
{
	if ( !m_labelingInfoDirty )
		return;

	m_labelingInfoDirty = false;

	if ( m_labelcostsAll )
	{
		if ( updateCounts )
		{
			memset(m_labelCounts,0,m_num_labels*sizeof(SiteID));
			for ( SiteID i = 0; i < m_num_sites; ++i )
				m_labelCounts[m_labeling[i]]++;
		}

		if ( updateActive )
		{
			for ( LabelCost* lc = m_labelcostsAll; lc; lc = lc->next )
				lc->active = false;

			EnergyType energy = 0;
			for ( LabelID l = 0; l < m_num_labels; ++l ) 
				if ( m_labelCounts[l] )
					for ( LabelCostIter* lci = m_labelcostsByLabel[l]; lci; lci = lci->next ) 
						lci->node->active = true;
		}
	}

	if ( updateCosts )
	{
		if (m_updateLabelingDataCosts)
			(this->*m_updateLabelingDataCosts)();
		else
			memset(m_labelingDataCosts,0,m_num_sites*sizeof(EnergyTermType));
	}
}

//-------------------------------------------------------------------
// Sets up the binary expansion energy, optimizes it, and updates the current labeling.
//
bool GCoptimization::alpha_expansion(LabelID alpha_label)
{
	if (alpha_label < 0)
		return false; // label was disabled due to setLabelOrder on subset of labels

	finalizeNeighbors();
	gcoclock_t ticks0 = gcoclock();

	if ( m_stepsThisCycleTotal == 0 )
		m_labelingInfoDirty = true; // if not inside expansion(), assume data cost function could have changed since last expansion
	updateLabelingInfo();

	// Determine list of active sites for this expansion move
	SiteID size = 0;
	SiteID *activeSites = new SiteID[m_num_sites];
	EnergyType afterExpansionEnergy = 0;
	try 
	{
		// Get list of active sites based on alpha and current labeling
		if ( m_queryActiveSitesExpansion )
			size = (this->*m_queryActiveSitesExpansion)(alpha_label,activeSites);
		if ( size == 0 )  // Nothing to do
		{
			delete [] activeSites;
			printStatus2(alpha_label,-1,size,ticks0);
			return false;
		}

		// Initialise reverse-lookup so that non-active neighbours can be identified
		// while constructing the graph
		for ( SiteID i = 0; i < size; i++ )
			m_lookupSiteVar[activeSites[i]] = i;

		// Create binary variables for each remaining site, add the data costs,
		// and compute the smooth costs between variables.
		EnergyT e(size+m_labelcostCount, // poor guess at number of pairwise terms needed :(
				 m_numNeighborsTotal+(m_labelcostCount?size+m_labelcostCount : 0),
				 handleError);
		e.add_variable(size);
		m_beforeExpansionEnergy = 0;
		if ( m_setupDataCostsExpansion   ) (this->*m_setupDataCostsExpansion  )(size,alpha_label,&e,activeSites);
		if ( m_setupSmoothCostsExpansion ) (this->*m_setupSmoothCostsExpansion)(size,alpha_label,&e,activeSites);
		EnergyType alphaCorrection = setupLabelCostsExpansion(size,alpha_label,&e,activeSites);
		checkInterrupt();
		afterExpansionEnergy = e.minimize() + alphaCorrection;
		checkInterrupt();

		if ( afterExpansionEnergy < m_beforeExpansionEnergy )
			(this->*m_applyNewLabeling)(&e,activeSites,size,alpha_label);

		for ( SiteID i = 0; i < size; i++ )
			m_lookupSiteVar[activeSites[i]] = -1; // restore m_lookupSite to all -1s

		printStatus2(alpha_label,-1,size,ticks0);
	} 
	catch (...)
	{
		delete [] activeSites;
		throw;
	}
	delete [] activeSites;
	return afterExpansionEnergy < m_beforeExpansionEnergy;
}

//-------------------------------------------------------------------

GCoptimization::EnergyType GCoptimization::oneExpansionIteration()
{
	permuteLabelTable();
	m_stepsThisCycle = 0;
	m_stepsThisCycleTotal = m_num_labels;

	// Each cycle is exactly one pass over the labels
	for (LabelID next = 0; next < m_num_labels; next++, m_stepsThisCycle++ )
		alpha_expansion(m_labelTable[next]);

	return compute_energy();
}

//-------------------------------------------------------------------//
//                  METHODS for SWAP MOVES                           //  
//-------------------------------------------------------------------//

GCoptimization::EnergyType GCoptimization::swap(int max_num_iterations)
{
	EnergyType new_energy,old_energy;
	if ( (this->*m_solveSpecialCases)(new_energy) )
		return new_energy;
	
	new_energy = compute_energy();
	old_energy = new_energy+1;
	printStatus1("starting alpha/beta-swap");

	if ( max_num_iterations == -1 )
		max_num_iterations = 10000000;
	int curr_cycle = 1;
	m_stepsThisCycleTotal = (m_num_labels*(m_num_labels-1))/2;
	try
	{
		while ( old_energy > new_energy && curr_cycle <= max_num_iterations)
		{
			gcoclock_t ticks0 = gcoclock();
			old_energy = new_energy;
			new_energy = oneSwapIteration();
			printStatus1(curr_cycle,true,ticks0);
			curr_cycle++;
		}
	} 
	catch (...)
	{
		m_stepsThisCycle = m_stepsThisCycleTotal = 0;
		throw;
	}
	m_stepsThisCycle = m_stepsThisCycleTotal = 0;

	return(new_energy);
}

//--------------------------------------------------------------------------------

GCoptimization::EnergyType GCoptimization::oneSwapIteration()
{
	LabelID next,next1;
	permuteLabelTable();
	m_stepsThisCycle = 0;

	for (next = 0;  next < m_num_labels;  next++ )
		for (next1 = m_num_labels - 1;  next1 >= 0;  next1-- )
			if ( m_labelTable[next] < m_labelTable[next1] )
			{
				alpha_beta_swap(m_labelTable[next],m_labelTable[next1]); 
				m_stepsThisCycle++;
			}

	return(compute_energy());
}

//---------------------------------------------------------------------------------

void GCoptimization::alpha_beta_swap(LabelID alpha_label, LabelID beta_label)
{
	assert( alpha_label >= 0 && alpha_label < m_num_labels && beta_label >= 0 && beta_label < m_num_labels);
	if ( m_labelcostsAll )
		handleError("Label costs only implemented for alpha-expansion.");

	finalizeNeighbors();
	gcoclock_t ticks0 = gcoclock();

	// Determine the list of active sites for this swap move
	SiteID size = 0;
	SiteID *activeSites = new SiteID[m_num_sites];
	try
	{
		for ( SiteID i = 0; i < m_num_sites; i++ )
		{
			if ( m_labeling[i] == alpha_label || m_labeling[i] == beta_label )
			{
				activeSites[size] = i;
				m_lookupSiteVar[i] = size;
				size++;
			}
		}
		if ( size == 0 )
		{
			delete [] activeSites;
			printStatus2(alpha_label,beta_label,size,ticks0);
			return;
		}

		// Create binary variables for each remaining site, add the data costs,
		// and compute the smooth costs between variables.
		EnergyT e(size,m_numNeighborsTotal,handleError);
		e.add_variable(size);
		if ( m_setupDataCostsSwap   ) (this->*m_setupDataCostsSwap  )(size,alpha_label,beta_label,&e,activeSites);
		if ( m_setupSmoothCostsSwap ) (this->*m_setupSmoothCostsSwap)(size,alpha_label,beta_label,&e,activeSites);
		checkInterrupt();
		e.minimize();
		checkInterrupt();
		
		// Apply the new labeling
		for ( SiteID i = 0; i < size; i++ )
		{
			m_labeling[activeSites[i]] = (e.get_var(i) == 0) ? alpha_label : beta_label;
			m_lookupSiteVar[activeSites[i]] = -1; // restore lookupSiteVar to all -1s
		}
		m_labelingInfoDirty = true;
	} 
	catch (...)
	{
		delete [] activeSites;
		throw;
	}
	delete [] activeSites;

	printStatus2(alpha_label,beta_label,size,ticks0);
}


//////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for the GCoptimizationGridGraph, derived from GCoptimization
////////////////////////////////////////////////////////////////////////////////////////////////////

GCoptimizationGridGraph::GCoptimizationGridGraph(SiteID width, SiteID height,LabelID num_labels)
						:GCoptimization(width*height,num_labels)
{
	assert( (width > 1) && (height > 1) && (num_labels > 1 ));

	m_weightedGraph = 0;
	for (int  i = 0; i < 4; i ++ )	m_unityWeights[i] = 1;

	m_width  = width;
	m_height = height;

	m_numNeighbors = new SiteID[m_num_sites];
	m_neighbors = new SiteID[4*m_num_sites];

	SiteID indexes[4] = {-1,1,-m_width,m_width};

	SiteID indexesL[3] = {1,-m_width,m_width};
	SiteID indexesR[3] = {-1,-m_width,m_width};
	SiteID indexesU[3] = {1,-1,m_width};
	SiteID indexesD[3] = {1,-1,-m_width};

	SiteID indexesUL[2] = {1,m_width};
	SiteID indexesUR[2] = {-1,m_width};
	SiteID indexesDL[2] = {1,-m_width};
	SiteID indexesDR[2] = {-1,-m_width};

	setupNeighbData(1,m_height-1,1,m_width-1,4,indexes);

	setupNeighbData(1,m_height-1,0,1,3,indexesL);
	setupNeighbData(1,m_height-1,m_width-1,m_width,3,indexesR);
	setupNeighbData(0,1,1,width-1,3,indexesU);
	setupNeighbData(m_height-1,m_height,1,m_width-1,3,indexesD);

	setupNeighbData(0,1,0,1,2,indexesUL);
	setupNeighbData(0,1,m_width-1,m_width,2,indexesUR);
	setupNeighbData(m_height-1,m_height,0,1,2,indexesDL);
	setupNeighbData(m_height-1,m_height,m_width-1,m_width,2,indexesDR);
}

//-------------------------------------------------------------------

GCoptimizationGridGraph::~GCoptimizationGridGraph()
{
	delete [] m_numNeighbors;
	if ( m_neighbors )
		delete [] m_neighbors;
	if (m_weightedGraph) delete [] m_neighborsWeights;
}


//-------------------------------------------------------------------

void GCoptimizationGridGraph::setupNeighbData(SiteID startY,SiteID endY,SiteID startX,
											  SiteID endX,SiteID maxInd,SiteID *indexes)
{
	SiteID x,y,pix;
	SiteID n;

	for ( y = startY; y < endY; y++ )
		for ( x = startX; x < endX; x++ )
		{
			pix = x+y*m_width;
			m_numNeighbors[pix] = maxInd;
			m_numNeighborsTotal += maxInd;

			for (n = 0; n < maxInd; n++ )
				m_neighbors[pix*4+n] = pix+indexes[n];
		}
}

//-------------------------------------------------------------------

void GCoptimizationGridGraph::finalizeNeighbors()
{
}

//-------------------------------------------------------------------

void GCoptimizationGridGraph::setSmoothCostVH(EnergyTermType *smoothArray, EnergyTermType *vCosts, EnergyTermType *hCosts)
{
	setSmoothCost(smoothArray);
	m_weightedGraph = 1;
	computeNeighborWeights(vCosts,hCosts);
}

//-------------------------------------------------------------------

void GCoptimizationGridGraph::giveNeighborInfo(SiteID site, SiteID *numSites, SiteID **neighbors, EnergyTermType **weights)
{
	*numSites  = m_numNeighbors[site];
	*neighbors = &m_neighbors[site*4];
	
	if (m_weightedGraph) *weights  = &m_neighborsWeights[site*4];
	else *weights = m_unityWeights;
}

//-------------------------------------------------------------------

void GCoptimizationGridGraph::computeNeighborWeights(EnergyTermType *vCosts,EnergyTermType *hCosts)
{
	SiteID i,n,nSite;
	GCoptimization::EnergyTermType weight;
	
	m_neighborsWeights = new EnergyTermType[m_num_sites*4];

	for ( i = 0; i < m_num_sites; i++ )
	{
		for ( n = 0; n < m_numNeighbors[i]; n++ )
		{
			nSite = m_neighbors[4*i+n];
			if ( i-nSite == 1 )            weight = hCosts[nSite];
			else if (i-nSite == -1 )       weight = hCosts[i];
			else if ( i-nSite == m_width ) weight = vCosts[nSite];
			else if (i-nSite == -m_width ) weight = vCosts[i];
	
			m_neighborsWeights[i*4+n] = weight;
		}
	}

}
////////////////////////////////////////////////////////////////////////////////////////////////
// Functions for the GCoptimizationGeneralGraph, derived from GCoptimization
////////////////////////////////////////////////////////////////////////////////////////////////////

GCoptimizationGeneralGraph::GCoptimizationGeneralGraph(SiteID num_sites,LabelID num_labels):GCoptimization(num_sites,num_labels)
{
	assert( num_sites > 1 && num_labels > 1 );

	m_neighborsIndexes = 0;
	m_neighborsWeights = 0;
	m_numNeighbors     = 0;
	m_neighbors        = 0;

	m_needTodeleteNeighbors        = true;
	m_needToFinishSettingNeighbors = true;
}

//------------------------------------------------------------------

GCoptimizationGeneralGraph::~GCoptimizationGeneralGraph()
{
		
	if ( m_neighbors )
		delete [] m_neighbors;

	if ( m_numNeighbors && m_needTodeleteNeighbors )
	{
		for ( SiteID i = 0; i < m_num_sites; i++ )
		{
			if (m_numNeighbors[i] != 0 )
			{
				delete [] m_neighborsIndexes[i];
				delete [] m_neighborsWeights[i];
			}
		}

		delete [] m_numNeighbors;
		delete [] m_neighborsIndexes;
		delete [] m_neighborsWeights;
	}
}

//------------------------------------------------------------------

void GCoptimizationGeneralGraph::finalizeNeighbors()
{
	if ( !m_needToFinishSettingNeighbors )
		return;
	m_needToFinishSettingNeighbors = false;

	Neighbor *tmp;
	SiteID i,site,count;

	EnergyTermType *tempWeights = new EnergyTermType[m_num_sites];
	SiteID *tempIndexes         = new SiteID[m_num_sites];
	
	if ( !tempWeights || !tempIndexes ) handleError("Not enough memory");

	m_numNeighbors     = new SiteID[m_num_sites];
	m_neighborsIndexes = new SiteID*[m_num_sites];
	m_neighborsWeights = new EnergyTermType*[m_num_sites];
	
	if ( !m_numNeighbors || !m_neighborsIndexes || !m_neighborsWeights ) handleError("Not enough memory.");

	for ( site = 0; site < m_num_sites; site++ )
	{
		if ( m_neighbors && !m_neighbors[site].isEmpty() )
		{
			m_neighbors[site].setCursorFront();
			count = 0;
			
			while ( m_neighbors[site].hasNext() )
			{
				tmp = (Neighbor *) (m_neighbors[site].next());
				tempIndexes[count] =  tmp->to_node;
				tempWeights[count] =  tmp->weight;
				delete tmp;
				count++;
			}
			m_numNeighbors[site]     = count;
			m_numNeighborsTotal     += count;
			m_neighborsIndexes[site] = new SiteID[count];
			m_neighborsWeights[site] = new EnergyTermType[count];
			
			if ( !m_neighborsIndexes[site] || !m_neighborsWeights[site] ) handleError("Not enough memory.");
			
			for ( i = 0; i < count; i++ )
			{
				m_neighborsIndexes[site][i] = tempIndexes[i];
				m_neighborsWeights[site][i] = tempWeights[i];
			}
		}
		else m_numNeighbors[site] = 0;

	}

	delete [] tempIndexes;
	delete [] tempWeights;
	if (m_neighbors) {
		delete [] m_neighbors;
		m_neighbors = 0;
	}
}
//------------------------------------------------------------------------------

void GCoptimizationGeneralGraph::giveNeighborInfo(SiteID site, SiteID *numSites, 
												  SiteID **neighbors, EnergyTermType **weights)
{
	if (m_numNeighbors) {
		(*numSites)  =  m_numNeighbors[site];
		(*neighbors) = m_neighborsIndexes[site];
		(*weights)   = m_neighborsWeights[site];
	} else {
		*numSites = 0;
		*neighbors = 0;
		*weights = 0;
	}
}


//------------------------------------------------------------------

void GCoptimizationGeneralGraph::setNeighbors(SiteID site1, SiteID site2, EnergyTermType weight)
{

	assert( site1 < m_num_sites && site1 >= 0 && site2 < m_num_sites && site2 >= 0);
	if ( m_needToFinishSettingNeighbors == false )
		handleError("Already set up neighborhood system.");

	if ( !m_neighbors )
	{
		m_neighbors = (LinkedBlockList *) new LinkedBlockList[m_num_sites];
		if ( !m_neighbors ) handleError("Not enough memory.");
	}

	Neighbor *temp1 = (Neighbor *) new Neighbor;
	Neighbor *temp2 = (Neighbor *) new Neighbor;

	temp1->weight  = weight;
	temp1->to_node = site2;

	temp2->weight  = weight;
	temp2->to_node = site1;

	m_neighbors[site1].addFront(temp1);
	m_neighbors[site2].addFront(temp2);
	
}
//------------------------------------------------------------------

void GCoptimizationGeneralGraph::setAllNeighbors(SiteID *numNeighbors,SiteID **neighborsIndexes,
												 EnergyTermType **neighborsWeights)
{
	m_needTodeleteNeighbors = false;
	m_needToFinishSettingNeighbors = false;
	if ( m_numNeighborsTotal > 0 )
		handleError("Already set up neighborhood system.");
	m_numNeighbors     = numNeighbors;
	m_numNeighborsTotal = 0;
	for (int site = 0; site < m_num_sites; site++ ) m_numNeighborsTotal += m_numNeighbors[site];
	m_neighborsIndexes = neighborsIndexes;
	m_neighborsWeights = neighborsWeights;
}



//------------------------------------------------------------------
// boring status messages

void GCoptimization::printStatus1(const char* extraMsg)
{
	if ( m_verbosity < 1 )
		return;
	if ( extraMsg )
		printf("gco>> %s\n",extraMsg);
	printf("gco>> initial energy: \tE=%lld (E=%lld+%lld+%lld)\n",(long long)compute_energy(),
		(long long)giveDataEnergy(), (long long)giveSmoothEnergy(), (long long)giveLabelEnergy()); 
	flushnow(); 
}

void GCoptimization::printStatus1(int cycle, bool isSwap, gcoclock_t ticks0)
{
	if ( m_verbosity < 1 )
		return;
	gcoclock_t ticks1 = gcoclock();
	printf("gco>> after cycle %2d: \tE=%lld (E=%lld+%lld+%lld);",cycle,(long long)compute_energy(),
		(long long)giveDataEnergy(),(long long)giveSmoothEnergy(),(long long)giveLabelEnergy());
	if ( m_stepsThisCycleTotal > 0 )
		printf(isSwap ? " \t%d swaps(s);" : " \t%d expansions(s);",m_stepsThisCycleTotal);
	if ( m_verbosity == 1 )
	{
		// Don't print time if time is already printed at finer scale, since printing
		// itself takes time (esp in MATLAB) and makes time useless at this level
		int ms = (int)(1000*(ticks1 - ticks0) / GCO_CLOCKS_PER_SEC);
		printf(" \t%d ms",ms);
	}
	printf("\n");
	flushnow(); 
}

void GCoptimization::printStatus2(int alpha, int beta, int numVars, gcoclock_t ticks0)
{
	if ( m_verbosity < 2 )
		return;
	int microsec = (int)(1000000*(gcoclock() - ticks0) / GCO_CLOCKS_PER_SEC);
	if ( beta >= 0 )
		printf("gco>>   after swap(%d,%d):",alpha+INDEX0,beta+INDEX0);
	else
		printf("gco>>   after expansion(%d):",alpha+INDEX0);
	printf(" \tE=%lld (E=%lld+%lld+%lld);\t %lld vars;",
		(long long)compute_energy(),(long long)giveDataEnergy(),
		(long long)giveSmoothEnergy(),(long long)giveLabelEnergy(),(long long)numVars);
	if ( m_stepsThisCycleTotal > 0 )
		printf(" \t(%d of %d);",m_stepsThisCycle+1,m_stepsThisCycleTotal);

	printf(microsec > 100 ? "\t %.2f ms\n" : "\t %.3f ms\n",(double)microsec/1000.0);
	flushnow();
}




//-------------------------------------------------------------------
// DataCostFnSparse methods
//-------------------------------------------------------------------


GCoptimization::DataCostFnSparse::DataCostFnSparse(SiteID num_sites, LabelID num_labels)
: m_num_sites(num_sites)
, m_num_labels(num_labels)
, m_buckets_per_label((m_num_sites + cSitesPerBucket-1)/cSitesPerBucket)
, m_buckets(0)
{
}

GCoptimization::DataCostFnSparse::DataCostFnSparse(const DataCostFnSparse& src)
: m_num_sites(src.m_num_sites)
, m_num_labels(src.m_num_labels)
, m_buckets_per_label(src.m_buckets_per_label)
, m_buckets(0)
{
	assert(!src.m_buckets); // not implemented
}

GCoptimization::DataCostFnSparse::~DataCostFnSparse()
{
	if (m_buckets) {
		for (LabelID l = 0; l < m_num_labels; ++l)
			if (m_buckets[l*m_buckets_per_label].begin)
				delete [] m_buckets[l*m_buckets_per_label].begin;
		delete [] m_buckets;
	}
}

void GCoptimization::DataCostFnSparse::set(LabelID l, const SparseDataCost* costs, SiteID count)
{
	// Create the bucket if necessary, and copy all the costs
	//
	if (!m_buckets) {
		m_buckets = new DataCostBucket[m_num_labels*m_buckets_per_label];
		memset(m_buckets, 0, m_num_labels*m_buckets_per_label*sizeof(DataCostBucket));
	}

	DataCostBucket* b = &m_buckets[l*m_buckets_per_label];
	if (b->begin)
		delete [] b->begin;
	SparseDataCost* next = new SparseDataCost[count];
	memcpy(next,costs,count*sizeof(SparseDataCost));

	//
	// Scan the list of costs and remember pointers to delimit the 'buckets', i.e. where 
	// ranges of SiteIDs lie along the array. Buckets can be empty (begin == end).
	//
	const SparseDataCost* end  = next+count;
	SiteID prev_site = -1;
	for (int i = 0; i < m_buckets_per_label; ++i) {
		b[i].begin = b[i].predict = next;
		SiteID end_site = (i+1)*cSitesPerBucket;
		while (next < end && next->site < end_site) {
			if (next->site < 0 || next->site >= m_num_sites)
				throw GCException("Invalid site id given for sparse data cost; must be within range.");
			if (next->site <= prev_site)
				throw GCException("Sparse data costs must be sorted in increasing order of SiteID");
			prev_site = next->site;
			++next;
		}
		b[i].end = next;
	}
}

GCoptimization::EnergyTermType GCoptimization::DataCostFnSparse::search(DataCostBucket& b, SiteID s)
{
	// Perform binary search for requested SiteID
	//
	const SparseDataCost* L = b.begin;
	const SparseDataCost* R = b.end-1;
	if ( R - L == m_num_sites )
		return b.begin[s].cost; // special case: this particular label is actually dense
	do {
		const SparseDataCost* mid = (const SparseDataCost*)((((size_t)L+(size_t)R) >> 1) & cDataCostPtrMask);
		if (s < mid->site)
			R = mid-1;         // eliminate upper range
		else if (mid->site < s)
			L = mid+1;         // eliminate lower range
		else {
			b.predict = mid+1;
			return mid->cost;  // found it!
		}
	} while (R - L > cLinearSearchSize);
	
	// Finish off with linear search over the remaining elements
	//
	do {
		if (L->site >= s) {
			if (L->site == s) {
				b.predict = L+1;
				return L->cost;
			}
			break;
		}
	} while (++L <= R);
	b.predict = L;

	return GCO_MAX_ENERGYTERM; // the site belongs to this bucket but with no cost specified
}

OLGA_INLINE GCoptimization::EnergyTermType GCoptimization::DataCostFnSparse::compute(SiteID s, LabelID l)
{
	DataCostBucket& b = m_buckets[l*m_buckets_per_label + (s >> cLogSitesPerBucket)];
	if (b.begin == b.end)
		return GCO_MAX_ENERGYTERM;
	if (b.predict < b.end) {
		// Check for correct prediction
		if (b.predict->site == s)
			return (b.predict++)->cost; // predict++ for next time
		
		// If the requested site is missing from the site ids near 'predict'
		// then we know it doesn't exist in the bucket, so return INF
		if (b.predict->site > s && b.predict > b.begin && (b.predict-1)->site < s)
			return GCO_MAX_ENERGYTERM;
	}
	if ( (size_t)b.end - (size_t)b.begin == cSitesPerBucket*sizeof(SparseDataCost) )
		return b.begin[s-b.begin->site].cost; // special case: this particular bucket is actually dense!

	return search(b,s);
}

GCoptimization::SiteID GCoptimization::DataCostFnSparse::queryActiveSitesExpansion(LabelID alpha_label, const LabelID* labeling, SiteID* activeSites)
{
	const SparseDataCost* next = m_buckets[alpha_label*m_buckets_per_label].begin;
	const SparseDataCost* end  = m_buckets[alpha_label*m_buckets_per_label + m_buckets_per_label-1].end;
	SiteID count = 0;
	for (; next < end; ++next) {
		if ( labeling[next->site] != alpha_label )
			activeSites[count++] = next->site;
	}
	return count;
}

