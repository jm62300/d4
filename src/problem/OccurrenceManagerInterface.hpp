#ifndef INTERFACE_OCC_MANAGER_INTERFACE
#define INTERFACE_OCC_MANAGER_INTERFACE

#include <vector>

class OccurrenceManagerInterface
{
public:
  std::vector< std::vector<int> > occList;
  virtual ~OccurrenceManagerInterface(){}

  virtual int getNbBinaryClause(unsigned v) = 0;
  virtual int getNbNotBinaryClause(unsigned v) = 0;
  virtual int getNbClause(unsigned v) = 0;
  virtual int getNbBinaryClause(int l) = 0;
  virtual int getNbNotBinaryClause(int l) = 0;
  virtual int getNbClause(int l) = 0;
  virtual int getNbClause() = 0;

  virtual std::vector<int> &getClause(int idx) = 0;
  virtual int getSizeClause(int idx){return getClause(idx).size();}
  virtual std::vector<int> &getVecIdxClause(int l) = 0;
  virtual int getNbUnsat(int idx) = 0;
  virtual int getNbunsignediable() = 0;
  virtual int getSumSizeClauses() = 0;

  virtual bool litIsAssigned(int l) = 0;
  virtual bool litIsAssignedToTrue(int l) = 0;

  virtual bool varIsAssigned(unsigned v) = 0;
  virtual int getMaxSizeClause() = 0;
  virtual bool isSatisfiedClause(int idx) = 0;
  virtual bool isSatisfiedClause(std::vector<int> &c) = 0;
  virtual bool isNotSatisfiedClauseAndInComponent(int idx, std::vector<bool> &inCurrentComponent) = 0;

  inline const std::vector< std::vector<int> > &getOccurrenceList(){return occList;}

  virtual int computeConnectedComponent(std::vector<std::vector<unsigned> > &varConnected, std::vector<unsigned> &setOfunsigned,
                                        std::vector<unsigned> &freeunsigned, std::vector<unsigned> &notFreeunsigned) = 0;
  virtual void preUpdate(std::vector<int> &lits) = 0;
  virtual void postUpdate(std::vector<int> &lits) = 0;
  virtual void initialize(std::vector<unsigned> &setOfunsigned, std::vector<int> &units) = 0;
  virtual void showOccurenceList() {}
  virtual void showFormula(){}
  virtual void initFormula(std::vector<std::vector<int> > &_clauses) = 0;

  virtual bool byPass(int mode, int idx){return false;}

  virtual void getCurrentClauses(std::vector<int> &idxClauses, std::vector<bool> &inCurrentComponent )
  {
    idxClauses.resize(0);
    for(int i = 0 ; i<getNbClause() ; i++)
      if(isNotSatisfiedClauseAndInComponent(i, inCurrentComponent)) idxClauses.push_back(i);
  }


  virtual void getCurrentClauses2(std::vector<int> &idxClauses, std::vector<bool> &inCurrentComponent )
  {
    idxClauses.resize(0);
    for(int i = 0 ; i<getNbClause() ; i++)
      if(isNotSatisfiedClauseAndInComponent(i, inCurrentComponent)) idxClauses.push_back(i);
  }


  virtual void updateCurrentClauseSet(std::vector<unsigned> &component){return;}
  virtual void popPreviousClauseSet(){return;}
};
#endif
