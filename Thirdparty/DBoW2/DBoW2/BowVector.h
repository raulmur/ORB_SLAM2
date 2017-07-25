/**
 * File: BowVector.h
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_BOW_VECTOR__
#define __D_T_BOW_VECTOR__

#include <iostream>
#include <map>
#include <vector>

#include "../DUtils/config.h"

namespace DBoW2 {

/// Id of words
typedef unsigned int WordId;

/// Value of a word
typedef double WordValue;

/// Id of nodes in the vocabulary treee
typedef unsigned int NodeId;

/// L-norms for normalization
EXPORT typedef enum LNorm
{
  L1,
  L2
} LNorm;

/// Weighting type
EXPORT typedef enum WeightingType
{
  TF_IDF,
  TF,
  IDF,
  BINARY
} WeightingType;

/// Scoring type
EXPORT typedef enum ScoringType
{
  L1_NORM,
  L2_NORM,
  CHI_SQUARE,
  KL,
  BHATTACHARYYA,
  DOT_PRODUCT,
} ScoringType;

/// Vector of words to represent images
/// stl的map结构，key为wordId，value为tfidf中的tf
class EXPORT BowVector: 
	public std::map<WordId, WordValue>
{
public:

	/** 
	 * Constructor
	 */
	BowVector(void);

	/**
	 * Destructor
	 */
	~BowVector(void);
	
	/**
	 * Adds a value to a word value existing in the vector, or creates a new
	 * word with the given value
	 * @param id word id to look for
	 * @param v value to create the word with, or to add to existing word
	 */
	void addWeight(WordId id, WordValue v);
	
	/**
	 * Adds a word with a value to the vector only if this does not exist yet
	 * @param id word id to look for
	 * @param v value to give to the word if this does not exist
	 */
	void addIfNotExist(WordId id, WordValue v);

	/**
	 * L1-Normalizes the values in the vector 
	 * @param norm_type norm used
	 */
	void normalize(LNorm norm_type);
	
	/**
	 * Prints the content of the bow vector
	 * @param out stream
	 * @param v
	 */
	friend std::ostream& operator<<(std::ostream &out, const BowVector &v);
	
	/**
	 * Saves the bow vector as a vector in a matlab file
	 * @param filename
	 * @param W number of words in the vocabulary
	 */
	void saveM(const std::string &filename, size_t W) const;
};

} // namespace DBoW2

#endif
