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
      // assert(OldD != ND && "list already contains decl");
      Node->Rest = OldD;
      Data.setPointer(Node);
      return;
    }
    // auto Vec = getLookupResult();
    // (void)Vec;
    // assert(llvm::find(Vec, ND) == Vec.end() && "list still contains decl");
    Node->Rest = Data.getPointer();
    Data.setPointer(Node);
  }

  template<typename Pred>
  void erase_if(Pred pred) {
    assert(getAsVector() && "Not in list mode!");
    ASTContext &C = getASTContext();
    for (Decls N = Data.getPointer(), Prev = N; N; ) {
      if (DeclsTy *L = N.dyn_cast<DeclsTy*>()) {
        if (pred(L->D)) {
          if (L == getAsVector())
            Data.setPointer(L->Rest);
          else
            Prev.get<DeclsTy*>()->Rest = L->Rest;
          N = L->Rest;
          C.DeallocateDeclListNode(L);
        } else {
          N = L->Rest;
          Prev = L;
        }
      } else {
        NamedDecl *ND = N.get<NamedDecl*>();
        if (pred(ND)) {
          if (getAsDecl())
            Data.setPointer(nullptr);
          else
            Prev.get<DeclsTy*>()->Rest = nullptr;
        }
        break;
      }
    }

    assert(llvm::find_if(getLookupResult(), pred) == getLookupResult().end() &&
           "Still exists!");
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
  DeclContext::lookup_result getLookupResult() const {
    if (isNull())
      return DeclContext::lookup_result();
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
    for (DeclsTy* N = getAsVector(); N; N = N->Rest.dyn_cast<DeclsTy*>()) {
      if (D->declarationReplaces(N->D, IsKnownNewer)) {
        N->D = D;
        return true;
      }
      if (auto *ND = N->Rest.dyn_cast<NamedDecl*>()) // the two element case
        if (D->declarationReplaces(ND, IsKnownNewer)) {
          N->Rest = ND;
          return true;
        }
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

  LLVM_DUMP_METHOD void dump() const {
    for (auto *ND : getLookupResult())
      llvm::errs() << ND->getNameAsString() << " " << ND << "\n";
    llvm::errs() << "---\n";
    for (Decls N = Data.getPointer(); N; ) {
      if (DeclsTy *L = N.dyn_cast<DeclsTy*>()) {
        llvm::errs() << L->D->getNameAsString() << " " << L->D << "\n";
        N = L->Rest;
      } else {
        NamedDecl *ND = N.get<NamedDecl*>();
        llvm::errs() << ND->getNameAsString() << " " << ND << "\n";
        break;
      }
    }
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
