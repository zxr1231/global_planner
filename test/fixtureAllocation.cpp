// Linked only into isolated fixture executables, never the planner library/nodes.
#include "fixtureAllocation.h"
#include <cstdlib>
#include <new>
#include <cstdint>
#include <algorithm>
namespace fixtureAllocation {thread_local Ledger* active=nullptr;}
namespace {
struct alignas(std::max_align_t) Header {size_t bytes;fixtureAllocation::Ledger* owner;};
void* allocate(size_t n){
 auto* owner=fixtureAllocation::active;if(n>std::numeric_limits<size_t>::max()-sizeof(Header))throw std::bad_alloc();
 size_t bytes=n+sizeof(Header);
 if(owner && (owner->current>owner->limit || bytes>owner->limit-owner->current)){owner->rejected=true;throw std::bad_alloc();}
 auto* h=static_cast<Header*>(std::malloc(bytes));if(!h)throw std::bad_alloc();h->bytes=bytes;h->owner=owner;
 if(owner){owner->current+=bytes;owner->peak=std::max(owner->peak,owner->current);++owner->allocations;}
 return h+1;
}
void release(void* p)noexcept {if(!p)return;auto* h=static_cast<Header*>(p)-1;if(h->owner)h->owner->current-=h->bytes;std::free(h);}
}
void* operator new(size_t n){return allocate(n);}void* operator new[](size_t n){return allocate(n);}
void operator delete(void* p)noexcept{release(p);}void operator delete[](void* p)noexcept{release(p);}
void operator delete(void* p,size_t)noexcept{release(p);}void operator delete[](void* p,size_t)noexcept{release(p);}
void* operator new(size_t n,const std::nothrow_t&)noexcept{try{return allocate(n);}catch(...){return nullptr;}}
void* operator new[](size_t n,const std::nothrow_t&)noexcept{try{return allocate(n);}catch(...){return nullptr;}}
void operator delete(void* p,const std::nothrow_t&)noexcept{release(p);}void operator delete[](void* p,const std::nothrow_t&)noexcept{release(p);}
