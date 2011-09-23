/** -*- mode: c++; c-indent-level: 4; c++-member-init-indent: 8; comment-column: 35; -*-

The above line is usefulin Emacs-like editors
 */

/*
Template for simple quadratic crossover operators
=================================================

Quadratic crossover operators modify the both genotypes
*/

#ifndef eoSRAutomaticPidTuningQuadCrossover_H
#define eoSRAutomaticPidTuningQuadCrossover_H

#include <eoOp.h>

/**
 *  Always write a comment in this format before class definition
 *  if you want the class to be documented by Doxygen
 *
 * THere is NO ASSUMPTION on the class GenoypeT.
 * In particular, it does not need to derive from EO
 */
template<class GenotypeT>
class eoSRAutomaticPidTuningQuadCrossover: public eoQuadOp<GenotypeT>
{
public:
  /**
   * Ctor - no requirement
   */
// START eventually add or modify the anyVariable argument
  eoSRAutomaticPidTuningQuadCrossover()
  //  eoSRAutomaticPidTuningQuadCrossover( varType  _anyVariable) : anyVariable(_anyVariable)
// END eventually add or modify the anyVariable argument
  {
    // START Code of Ctor of an eoSRAutomaticPidTuningEvalFunc object
    // END   Code of Ctor of an eoSRAutomaticPidTuningEvalFunc object
  }

  /// The class name. Used to display statistics
  string className() const { return "eoSRAutomaticPidTuningQuadCrossover"; }

  /**
   * eoQuad crossover - _genotype1 and _genotype2 are the (future)
   *       offspring, i.e. _copies_ of the parents, to be modified
   * @param _genotype1 The first parent
   * @param _genotype2 The second parent
   */
  bool operator()(GenotypeT& _genotype1, GenotypeT & _genotype2)
  {
      bool oneAtLeastIsModified(true);
    // START code for crossover of _genotype1 and _genotype2 objects

       /** Requirement
	* if (at least one genotype has been modified) // no way to distinguish
	*     oneAtLeastIsModified = true;
	* else
	*     oneAtLeastIsModified = false;
	*/
    return oneAtLeastIsModified;
    // END code for crossover of _genotype1 and _genotype2 objects
  }

private:
// START Private data of an eoSRAutomaticPidTuningQuadCrossover object
  //  varType anyVariable;		   // for example ...
// END   Private data of an eoSRAutomaticPidTuningQuadCrossover object
};

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif
