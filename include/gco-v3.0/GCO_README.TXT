##############################################################################
#                                                                            #
#    GCoptimization - software for energy minimization with graph cuts       #
#                        Version 3.0                                         #
#    http://www.csd.uwo.ca/faculty/olga/software.html                        #
#                                                                            #
#    Copyright 2007-2010 Olga Veksler  <olga@csd.uwo.ca>                     #
#                        Andrew Delong <andrew.delong@gmail.com>             #
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

2. License & disclaimer.

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


3. Energy Minimization

This software is for minimizing sums of three types of terms:

   E(labeling) = DataCosts(labeling) + SmoothCosts(labeling) + LabelCosts(labeling).

More specifically,
   
   DataCosts(l)   = sum_p  D_p(l_p)      where l_p is a potential label for site p,
                                         and D_p is the cost of assigning l_p,
   
   SmoothCosts(l) = sum_pq V_pq(l_p,l_q) where p and q are two distinct site, and
                                         V_pq is the cost for assigning l_p, l_q, and
   
   LabelCosts(l)  = sum_L'  h_L'(l)      where L' is some subset of labels L, and h_L'
                                         adds a cost iff at least one label from L'
                                         appears in the labeling l.

Here we have a finite set of sites (or pixels) P and a finite set of labels L.
A labeling l is assignments of labels in L to pixels in P. The individual pixels 
are referred to with small letters p and q, label of pixel p is denoted by l_p,
and the set of all label-pixel assignments is denoted by l, that is
l = {l_p | p in P}.

The first term in the energy function E(l) is typically called the data term, and
it consists of the sum over all pixels p of the penalty(or cost) D(p,l_p), what
should be the cost of assigning label l_p to pixel p.  D(p,l_p) can be arbitrary.

The second term is a sum over all pairs of neighboring pixels {p,q}. 
That is there is a neighborhood relation on the set of pixels (this relationship
is symmetric, that is if p is a neighbor of q then q is a neighbor of p).
Here we assume that the neighbor pairs are unordered. This means that if pixels p and q are 
neighbors, then there is either Vpq(l_p,l_q) in the second sum of the energy,
or Vqp(l_q,l_p), but not both. This is not a restriction, since in practice, one can always go
from the ordered energy to the unordered one.  This second term is typically called the smoothness
term.

The third term is a sum over all labels (or, more generally, all subsets of labels)
such that each label can have non-negative penalties associated with its use.
This "label cost" feature is used to encourage labelings that use as fewer unique labels
or, more generally, labels from as few unique subsets as possible.

The expansion algorithm for energy minimization can be used whenever for any 3 labels a,b,c
V(a,a) + V(b,c) <= V(a,c)+V(b,a). In other words, expansion algorithm can be used if
the binary energy for the expansion algorithm step is regular, using V. Kolmogorov's terminology.

The swap algorithm for energy minimization can be used whenever for any 2 labels a,b
V(a,a) + V(b,b) <= V(a,b)+V(b,a). In other words, swap algorithm can be used if
the binary energy for the swap algorithm step is regular, using V. Kolmogorov's terminology.

##################################################################

4. Data Types

Inside the GCoptimization.h file you can customize the following typedefs:

typedef  <numeric type>  SiteID;          // index of a site (pixel); default is int32
typedef  <numeric type>  LabelID;         // index of a label;        default is int32
typedef  <numeric type>  EnergyType;      // total energy value;      default is int64
typedef  <numeric type>  EnergyTermType;  // individual energy term;  default is int32

For efficiency it is best to use integral types when possible; be sure that they are
large enough to avoid integer overflow. By default, the library will warn if energy 
terms are dangerously large (larger than GCO_MAX_ENERGYTERM=10000000). Even with float
types it's best to avoid extremely large energy terms, because it introduces rounding
errors, and the behaviour of the algorithm is undefined if Inf or NaN values appear
in the energy terms.

Keep in mind that float/double may cause expansion/swap to report small increase in energy, 
due to arithmetic error during max-flow computation.

###########################################################################

5. Specifying the energy

Before optimizing the energy, one has to specify it, that is specify the number of
labels, number of pixels, neighborhood system, the data terms, and the smoothness terms.
There are 2  constructors to use, one in case of the grid graph, and another in case
of a general graph.
In all cases, it is assumed that the sites go between 0...num_sites-1, 
and labels go between  0....num_labels-1.
For a grid (4-connected) graph, site at coordinates (x,y) is numbered with x+y*width, where width
is the width of the grid, that is the row-major ordering of arrays is assumed. 
________________________________________________________________________________________________

Constructor A.

GCoptimizationGridGraph(int width, int height,int num_labels);

Use this constructor only for grid of size width by height.  If you use this constructor, 
4 connected grid neigbhorhood structure is assumed, so you don't need to specify neighborhood
structure separately (and indeed cannot do so). 
_______________________________________________________________________________________________

Constructor B.

GCoptimizationGeneralGraph(int num_sites,int num_labels);


Use this constructor for general graphs.  If you use this constructor, you must setup up
neighborhood system using function. You can either specify neighbors individually or all at once.

i) setNeighbors(SiteID s1, SiteID s2, EnergyTermType weight=1);
Specifies neighbors individually. You must call this function exactly once  for any 
pair of neighboring sites s1 and s2. That is if you call setNeighbors(s1,s2) then you should not call 
setNeighbors(s2,s1).   If Vpq(l_p,l_q) = V(l_p,l_q)*w_pq, where V(l_p,l_q) is some function that
depends only on the labels l_p,l_q, then specify w_pq by using: setNeighbors(p,q,w_pq). 

ii) To pass in all neighbor information at once, use function:

void setAllNeighbors(SiteID *numNeighbors,SiteID **neighborsIndexes,EnergyTermType **neighborsWeights);
Here:
    (a) numNeighbors is an array of size num_sites, and numNeighbors[i] is the number of neighbors for site i

    (b) neighborIndexes is an array of size num_pixels which stores of pointers. Namely, 
        neighborsIndexes[i] is a pointer to the array storing the sites which are neighbors to site i

    (c) neighborWeights is an array of size num_sites, and neighborWeighs[i] is a pointer to array 
        storing the weights between site i and its neighbors in the same order as neighborIndexes[i] 
        stores the indexes of neighbors. Example: if sites i and j are neighbors, then 
        for some k and m, neighborsIndexes[i][k] == j and neighborsIndexes[j][m] = i. Then
        neighborWeights[i][k] = w_ij and neighborWeights[j][m] = w_ij, where w_ij is the weight
        betwen neighbors i and j, that is V_ij = w_ij *V(l_i,l_j)


_______________________________________________________________________________________________


6. Setting the data costs, smooth costs, and label costs.

The following functions can be called any time before or after expansion.


------------------------Data Costs (unary terms)-----------------------

(a) void setDataCost(EnergyTermType *dataArray);
    dataArray is an array s.t. the data cost for pixel p and  label l is stored at                        
    dataArray[pixel*num_labels+l].  If the current neighborhood system is a grid, then                    
    the data term for label l and pixel with coordinates (x,y) is stored at                               
    dataArray[(x+y*width)*num_labels + l]. Thus the size of array dataArray is num_pixels*num_labels.
    Can call this function only one time. 

(b) void setDataCost(DataCostFn fn); 
    DataCostFn is a pointer to a function  f(Pixel p, Label l), s.t. the data cost for pixel p to have      
    label l  is given by f(p,l). Can call this function only one time.

(c) void setDataCost(DataCostFnExtra fn,void *extraData);
    DataCostFnExtra is a pointer to a function  f(SiteID p, LabelID l,void *extraData), s.t. the data 
    cost for pixel p to have label l  is given by f(p,l,extraData).  Can call this function only one time.

(d) void setDataCost(SiteID s, LabelID l, EnergyTermType e); 
    sets up D(s,l) = 3; You must call this function for each pixel and each label.

(e) void setDataCostFunctor(DataCostFunctor* f);
    Experienced C++ users can subclass our DataCostFunctor base class to achieve
    a similar functionality as (b) or (c) above. By overriding the compute() method 
    of DataCostFunctor, your compute() method will be called by the GCoptimization
    class each time a data penalty must be computed.
         
(f) struct SparseDataCost {
        SiteID site;
        EnergyTermType cost;
    };
    void setDataCost(LabelID l, SparseDataCost *costs, SiteID count);
    
    For some applications, each label is feasible for only a small fraction
    of the overall sites. One way to do this is to simply assign high cost to
    any infeasible site. A much more efficient way is to specify exactly
    which sites are feasible for each label. Do this by calling
    setDataCost(label, costs_for_label, num_costs_for_label) once for each
    label. The cost array will be copied internally, so your cost array
    can be freed.
    Note that giveDataEnergy() will add a huge constant for each site
    that is assigned an infeasible label in the current labeling.
    

------------------------Smooth Costs (pairwise terms)-----------------------

(a) void setSmoothCost(EnergyTermType *V)
    
    V is an array of smoothness costs, such that V_pq(label1,label2)  is stored at V[label1+num_labels*label2]        
    If graph is a grid, then using this  function only if the smooth costs are not spacially varying    
    that is the smoothness penalty V depends only on labels, but not on sites.  If the graph is 
    not a grid, then you can specify spacially varying coefficients w_pq when you set up the
    neighborhood system using setNeighbor(p,q,w_pq) function. In this case, 
    V_pq(label1,label2) =  V[label1+num_labels*label2]*w_pq. This function can be called only one time.                            

(b) void setSmoothCost(SmoothCostFn fn);

    fn is pointer to a function f(s1,s2,l1,l2) such that smoothness penalty for neigboring sites     
    s1 and s2 to  have labels, respectively, l1 and l2 is f(s1,s2,l1,l2). This function can be 
    called only one time.         

(c) void setSmoothCost(SmoothCostFnExtra fn,void *extraData);	

    Same as above, but can pass an extra pointer to the data needed for computation

(d) void setSmoothCost(LabelID l1, LabelID l2, EnergyTermType e)

    sets up V(l1,l2) = e. Must call this function for each pair of labels (l1,l2). Notice
    that for any l1 and l2, you must call this function on (l1,l2) AND (l2,l1).
    V(l1,l2) has to be equal to V(l2,l1) in this case.

(e) void setSmoothCostVH(EnergyTermType *V, EnergyTermType *vCosts, EnergyTermType *hCosts);

    This function should be used only if the graph is a grid (GCoptimizationGridGraph class).
    Array V is the same as above, under (a).
    Arrays hCosts and vCosts have the same size as the image (that is width*height), and are used to set
    the spatially varying coefficients w_pq. If p = (x,y) and q = (x+1,y), then 
    w_pq =  hCosts[x+y*width], and so the smoothness penalty for pixels (x,y) and (x+1,y) to have labels       
    label1 and label2, that is V_pq(label1,label2) = V[label1+num_labels*label2]*hCosts[x+y*width]
    If p = (x,y) and q = (x,y+q), then 
    w_pq =  vCosts[x+y*width], and so the smoothness penalty for pixels (x,y) and (x,y+1) to have labels       
    label1 and label2, that is V_pq(label1,label2) = V[label1+num_labels*label2]*vCosts[x+y*width]
    This function can be only called one time.

(f) void setSmoothCostFunctor(SmoothCostFunctor* f);

    Experienced C++ users can subclass our SmoothCostFunctor base class to achieve
    a similar functionality as (b) or (c) above. By overriding the compute() method 
    of SmoothCostFunctor, your compute() method will be called by the GCoptimization
    class each time a smoothness penalty must be computed.


------------------------Label Costs (global indicator potentials)--------------------

(a) void setLabelCost(EnergyTermType cost);
    Penalize the appearance of all labels equally. Replaces all current label
    costs, if any.

(b) void setLabelCost(EnergyTermType* costArray);
    Set each individual label cost separately. The costArray must have one
    entry for each possible label. Replaces all current label costs, if any.
    The cost array will be copied intenally, so your array can be freed.

(c) void setLabelSubsetCost(LabelID* labels, LabelID numLabels, EnergyTermType cost);
    Set cost for a specific subset of labels. The cost will be imposed iff the
    current labeling contains at least one label from the 'labels' array.
    The labels array will be copied internally, so your array can be freed.

##################################################################

6. Optimizing the energy

You can optimize the energy and get the resulting labeling using the following functions.  Notice that they can 
be called as many times as one wishes after the constructor has been called and the data/smoothness terms
(and the neighborhood system, if general graph) has beeen set.  The initial labeling is set to consists of
all 0's. Use function setLabel(SiteID pixelP, LabelID labelL), described under heading (x) in this section
to initialize the labeling to anything else (but in the valid range, of course, labels must be between
0 and num_labels-1)

a) EnergyType expansion(int max_num_iterations=-1);
   Runs the expansion algorithm until convergence (convergence is guaranteed)
   or, if max_num_iterations > 0, until a certain number of cycles (iterations).
   Returns the energy of the resulting labeling.

b) bool alpha_expansion(LabelID alpha_label);
   Performs expansion on the label specified by alpha_label. 
   Returns true if the energy was decreased, false otherwise.

c) EnergyType swap(int max_num_iterations=-1);
   Runs the alpha-beta swap algorithm until convergence (convergence is guaranteed)
   or, if max_num_iterations > 0, until a certain number of cycles (iterations).
   Returns the energy of the resulting labeling.

d) void alpha_beta_swap(LabelID alpha_label, LabelID beta_label);
   Performs  swap on a pair of labels, specified by the input parameters alpha_label, beta_label.

e) EnergyType compute_energy();
   EnergyType giveDataEnergy();
   EnergyType giveSmoothEnergy();
   EnergyType giveLabelEnergy();
   Returns respectively the total, data part, smooth part, and label part of the energy of the current labling.

f) LabelID whatLabel(SiteID site);
   Returns the current label assigned to site.  Can be called at any time after the constructor call.

g) void setLabel(SiteID s, LabelID l);
   Sets the label of site s to the the input parameter l. Can be called at any time after 
   the constructor call. This is useful for initializing the labeling to something specific 
   before optimization starts.

h) void setLabelOrder(bool RANDOM_LABEL_ORDER);
   By default, the labels for the swap and expansion algorithms are visited in not random order, 
   but random label visitation might give better results. To set the label order to
   be not random, call setLabelOrder(false).  To set it to be random, call setLabelOrder(true). Notice,
   that by using functions under heading (iii) and (vii) you can completely  and exactly specify the desired
   order on labels.

##################################################################

7. Example usage.

   See example.cpp for C++ example, or see matlab\README.TXT for MATLAB example.

