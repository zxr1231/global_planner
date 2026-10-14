#pragma once
#include <cstddef>
#include <limits>
namespace fixtureAllocation {
struct Ledger {size_t current=0,peak=0,limit=256u*1024u*1024u,allocations=0;bool rejected=false;};
extern thread_local Ledger* active;
struct Scope {
 Ledger* previous;
 explicit Scope(Ledger& l):previous(active){active=&l;}
 ~Scope(){active=previous;}
};
}
