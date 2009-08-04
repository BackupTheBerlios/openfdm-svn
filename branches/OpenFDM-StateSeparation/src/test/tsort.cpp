#include <vector>
#include <list>
#include <iostream>
#include <cstdlib>

#include <OpenFDM/TopologySort.h>

using namespace OpenFDM;

struct Test {
  Test(unsigned i) :
    id(i)
  { }
  Test(unsigned i, unsigned d) :
    id(i)
  { depends.push_back(d); }
  Test(unsigned i, unsigned d1, unsigned d2) :
    id(i)
  { depends.push_back(d1); depends.push_back(d2); }
  Test(unsigned i, unsigned d1, unsigned d2, unsigned d3) :
    id(i)
  { depends.push_back(d1); depends.push_back(d2); depends.push_back(d3); }

  void addDependency(unsigned d)
  { depends.push_back(d); }

  bool dependsOn(const Test& other) const
  {
    for (unsigned i = 0; i < depends.size(); ++i)
      if (depends[i] == other.id)
        return true;
    return false;
  }

  unsigned id;
  std::vector<unsigned> depends;
};

std::ostream&
operator<<(std::ostream& s, const Test& t)
{
  s << "(" << t.id;
  for (unsigned i = 0; i < t.depends.size(); ++i)
    s << " " << t.depends[i];
  return s << ")";
}

template<typename T>
std::ostream&
operator<<(std::ostream& s, const std::list<T>& l)
{
  for (typename std::list<T>::const_iterator i = l.begin(); i != l.end(); ++i)
    s << " " << *i;
  return s;
}

struct Depends {
  bool operator()(const Test& t0, const Test& t1) const
  { return t0.dependsOn(t1); }
};

template<typename T, typename D>
inline bool
verify(const std::list<T>& sortedList, const D& depends)
{
  for (typename std::list<T>::const_iterator i = sortedList.begin();
       i != sortedList.end(); ++i) {
    for (typename std::list<T>::const_iterator j = i;
         j != sortedList.end(); ++j) {
      if (depends(*i, *j))
        return false;
    }
  }
  return true;
}

template<typename T, typename D>
bool
testrun(std::list<T>& source, const D& depends)
{
  std::list<T> orig = source;
  std::list<T> sorted;
  tsort(sorted, source, depends);
  if (verify(sorted, depends))
    return true;
  std::cout << "---- FAIL ----" << std::endl;
  std::cout << "Original:" << orig << std::endl;
  std::cout << "Sorted:" << sorted << std::endl;
  std::cout << "Cycles:" << source << std::endl;
  std::cout << std::endl;
  return false;
}

bool
randomTest(unsigned num, unsigned maxNumDeps)
{
  // Build up a direct acyclic graph
  std::vector<Test> source0;
  source0.reserve(num);
  for (unsigned i = 0; i < num; ++i) {
    Test t(i);
    unsigned numdeps = rand() % (maxNumDeps + 1);
    if (i < numdeps)
      numdeps = i;
    for (unsigned j = 0; j < numdeps; ++j)
      t.addDependency(rand()%i);
    source0.push_back(t);
  }

  // Build a permutation of 0:num-1
  std::vector<unsigned> perm(num);
  for (unsigned i = 0; i < num; ++i)
    perm[i] = i;
  for (unsigned i = 0; i < num*num; ++i)
    std::swap(perm[rand()%num], perm[rand()%num]);

  // Now permute the dag
  std::list<Test> source;
  for (unsigned i = 0; i < num; ++i)
    source.push_back(source0[perm[i]]);

  // Test if the algorithm works
  return testrun(source, Depends());
}

int main()
{
  std::list<Test> hmm;

  /////////////////////////////////////////////
  hmm.clear();
  hmm.push_back(Test(0, 0));

  if (!testrun(hmm, Depends()))
    return EXIT_FAILURE;

  /////////////////////////////////////////////
  hmm.clear();
  hmm.push_back(Test(0, 1));
  hmm.push_back(Test(1, 0));

  if (!testrun(hmm, Depends()))
    return EXIT_FAILURE;

  /////////////////////////////////////////////
  /// Breaks at pure depth first
  hmm.clear();
  hmm.push_back(Test(3, 1, 2));
  hmm.push_back(Test(2, 1));
  hmm.push_back(Test(1, 0));
  hmm.push_back(Test(0));

  if (!testrun(hmm, Depends()))
    return EXIT_FAILURE;

  /////////////////////////////////////////////
  /// Breaks at pure broad first
  hmm.clear();
  hmm.push_back(Test(3, 2, 1));
  hmm.push_back(Test(1, 0));
  hmm.push_back(Test(2, 1));
  hmm.push_back(Test(0));

  if (!testrun(hmm, Depends()))
    return EXIT_FAILURE;

  /////////////////////////////////////////////
  /// Breaks with bread first and depth first
  hmm.clear();
  hmm.push_back(Test(5, 3, 4));
  hmm.push_back(Test(4, 1));
  hmm.push_back(Test(3, 1, 2));
  hmm.push_back(Test(2, 1));
  hmm.push_back(Test(1, 0));
  hmm.push_back(Test(0));

  if (!testrun(hmm, Depends()))
    return EXIT_FAILURE;


  /////////////////////////////////////////////
  /// Breaks ???
  hmm.clear();
  hmm.push_back(Test(3, 0, 2));
  hmm.push_back(Test(0));
  hmm.push_back(Test(2, 1));
  hmm.push_back(Test(1, 0));

  if (!testrun(hmm, Depends()))
    return EXIT_FAILURE;

  /// Random tests ...
  unsigned numTests = 100;
  for (unsigned maxNumDeps = 1; maxNumDeps < 10; ++maxNumDeps) {
    for (unsigned num = 2; num < 100; ++num) {
      for (unsigned count = 0; count < numTests; ++count) {
        if (!randomTest(num, maxNumDeps))
          return EXIT_FAILURE;
      }
    }
  }

  return EXIT_SUCCESS;
}
