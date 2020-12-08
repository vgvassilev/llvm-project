//===- DeclContextInternals.h - DeclContext Representation ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//  This file defines the data structures used in the implementation
//  of DeclContext.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_AST_DECLCONTEXTINTERNALS_H
#define LLVM_CLANG_AST_DECLCONTEXTINTERNALS_H

#include "clang/AST/ASTContext.h"
#include "clang/AST/Decl.h"
#include "clang/AST/DeclBase.h"
#include "clang/AST/DeclCXX.h"
#include "clang/AST/DeclarationName.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/PointerIntPair.h"
#include "llvm/ADT/PointerUnion.h"
#include <algorithm>
#include <cassert>

namespace clang {

class DependentDiagnostic;

/// An array of decls optimized for the common case of only containing
/// one entry.
class StoredDeclsList {
  using DeclsTy = DeclListNode;
  using Decls = DeclListNode::Decls;

  /// A collection of declarations, with a flag to indicate if we have
  /// further external declarations.
  using DeclsAndHasExternalTy = llvm::PointerIntPair<Decls, 1, bool>;

  /// The stored data, which will be either a pointer to a NamedDecl,
  /// or a pointer to a vector with a flag to indicate if there are further
  /// external declarations.
  DeclsAndHasExternalTy Data;

  void setOnlyValue(NamedDecl *ND) {
    assert(!getAsVector() && "Not inline");
    Data.setPointer(ND);
    Data.setInt(0);
    // Make sure that Data is a plain NamedDecl* so we can use its address
    // at getLookupResult.
    assert(*(NamedDecl **)Data.getPointer().getAddrOfPtr1() == ND &&
           "PointerUnion mangles the NamedDecl pointer!");
  }

  void push_front(NamedDecl *ND) {
    assert(!isNull() && "No need to call push_front");

    ASTContext &C = getASTContext();
    DeclsTy *Node = C.AllocateDeclListNode(ND);

    // If this is the second decl added optimize the use by setting the
    // pointer to the next to point to OldD. This way we save an extra
    // allocation.
    if (NamedDecl *OldD = getAsDecl()) {
      // FIXME: Enable both asserts...
      //assert(OldD != ND && "list already contains decl");
      Node->Rest = OldD;
      Data.setPointer(Node);
      return;
    }

    // assert(llvm::find(*getAsVector(), ND) == std::end(*getAsVector()) &&
    //        "list still contains decl");
    Node->Rest = Data.getPointer();
    Data.setPointer(Node);
  }

  template<typename Pred>
  void erase_if(Pred pred) {
    DeclsTy *Prev = getAsVector();
    assert(Prev && "Not in list mode!");

    ASTContext &C = getASTContext();
    DeclsTy *Next = Prev;
    while(Next->Rest.dyn_cast<DeclsTy*>()) {
      if (pred(Next->D)) {
        Prev->Rest = Next->Rest;
        // FIXME: Move after assigning from Next.Rest
        //C.DeallocateDeclListNode(Next);
      }

      Next = Next->Rest.get<DeclsTy*>();
      if (Next->Rest.dyn_cast<DeclsTy*>())
        Prev = Next;
    }

    // The last element's Rest points to a NamedDecl to save space.
    NamedDecl *ND = Next->Rest.get<NamedDecl*>();
    if (Prev == Next) {
      // Switch to single element if only one item in the list is left.
      if (pred(ND)) {
        Data.setPointer(Next->D);
        C.DeallocateDeclListNode(Next);
      } else if (pred(Next->D)) {
        Data.setPointer(ND);
        C.DeallocateDeclListNode(Next);
      }
    } else {
      if (pred(ND)) {
        Prev->Rest = Next->D;
        C.DeallocateDeclListNode(Next);
      } else if (pred(Next->D)) {
        Prev->Rest = ND;
        C.DeallocateDeclListNode(Next);
      }
    }
  }

  void erase(NamedDecl *ND) {
    erase_if([ND](NamedDecl *D){ return D == ND; });
  }

public:
  StoredDeclsList() = default;

  StoredDeclsList(StoredDeclsList &&RHS) : Data(RHS.Data) {
    RHS.Data.setPointer(nullptr);
    RHS.Data.setInt(0);
  }

  ~StoredDeclsList() {
    // If this is a vector-form, free the vector.
    if (DeclsTy *Vector = getAsVector()) {
      ASTContext &C = getASTContext();
      C.DeallocateDeclListNode(Vector);
    }
  }

  StoredDeclsList &operator=(StoredDeclsList &&RHS) {
    if (DeclsTy *Vector = getAsVector()) {
      ASTContext &C = getASTContext();
      C.DeallocateDeclListNode(Vector);
    }
    Data = RHS.Data;
    RHS.Data.setPointer(nullptr);
    RHS.Data.setInt(0);
    return *this;
  }

  bool isNull() const { return Data.getPointer().isNull(); }

  ASTContext &getASTContext() {
    assert(!isNull() && "No ASTContext.");
    if (NamedDecl *ND = getAsDecl())
      return ND->getASTContext();
    return getAsVector()->D->getASTContext();
  }

  DeclsAndHasExternalTy getAsVectorAndHasExternal() const {
    return Data;
  }

  NamedDecl *getAsDecl() const {
    return getAsVectorAndHasExternal().getPointer().dyn_cast<NamedDecl *>();
  }

  DeclsTy *getAsVector() const {
    return getAsVectorAndHasExternal().getPointer().dyn_cast<DeclsTy*>();
  }


  bool hasExternalDecls() const {
    return getAsVectorAndHasExternal().getInt();
  }

  void setHasExternalDecls() {
    Data.setInt(1);
  }

  void remove(NamedDecl *D) {
    assert(!isNull() && "removing from empty list");
    if (NamedDecl *Singleton = getAsDecl()) {
      if (Singleton == D)
        Data.setPointer(nullptr);
      return;
    }

    erase(D);
  }

  /// Remove any declarations which were imported from an external
  /// AST source.
  void removeExternalDecls() {
    if (isNull()) {
      // Nothing to do.
    } else if (NamedDecl *Singleton = getAsDecl()) {
      if (Singleton->isFromASTFile())
        *this = StoredDeclsList();
    } else {
      erase_if([](NamedDecl *ND){ return ND->isFromASTFile(); });
      // Don't have any external decls any more.
      Data.setInt(0);
    }
  }

  /// getLookupResult - Return an array of all the decls that this list
  /// represents.
  DeclContext::lookup_result getLookupResult() {
    return DeclContext::lookup_result(Data.getPointer());
  }

  /// HandleRedeclaration - If this is a redeclaration of an existing decl,
  /// replace the old one with D and return true.  Otherwise return false.
  bool HandleRedeclaration(NamedDecl *D, bool IsKnownNewer) {
    // Most decls only have one entry in their list, special case it.
    if (NamedDecl *OldD = getAsDecl()) {
      if (!D->declarationReplaces(OldD, IsKnownNewer))
        return false;
      setOnlyValue(D);
      return true;
    }

    // Determine if this declaration is actually a redeclaration.
    DeclsTy &Vec = *getAsVector();
    while (DeclsTy *Next = Vec.Rest.dyn_cast<DeclsTy*>()) {
      if (D->declarationReplaces(Vec.D, IsKnownNewer)) {
        Vec.D = D;
        return true;
      }
      Vec = *Next;
    }

    // Handle the two element case.
    if (D->declarationReplaces(Vec.Rest.get<NamedDecl*>(), IsKnownNewer)) {
      Vec.Rest = D;
      return true;
    }

    if (D->declarationReplaces(Vec.D, IsKnownNewer)) {
      Vec.D = D;
      return true;
    }

    return false;
  }

  /// AddDecl - Called to add declarations when it is not a redeclaration to
  /// merge it into the appropriate place in our list.
  void AddDecl(NamedDecl *D) {
    if (isNull()) {
      setOnlyValue(D);
      return;
    }

    push_front(D);
  }
};

class StoredDeclsMap
    : public llvm::SmallDenseMap<DeclarationName, StoredDeclsList, 4> {
  friend class ASTContext; // walks the chain deleting these
  friend class DeclContext;

  llvm::PointerIntPair<StoredDeclsMap*, 1> Previous;
public:
  static void DestroyAll(StoredDeclsMap *Map, bool Dependent);
};

class DependentStoredDeclsMap : public StoredDeclsMap {
  friend class DeclContext; // iterates over diagnostics
  friend class DependentDiagnostic;

  DependentDiagnostic *FirstDiagnostic = nullptr;
public:
  DependentStoredDeclsMap() = default;
};

} // namespace clang

#endif // LLVM_CLANG_AST_DECLCONTEXTINTERNALS_H
