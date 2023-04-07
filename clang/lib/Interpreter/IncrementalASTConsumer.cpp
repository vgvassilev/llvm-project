//===--- IncrementalExecutor.cpp - Incremental AST Consumer ----------*- C++
//-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the class which performs incremental code generation.
//
//===----------------------------------------------------------------------===//

#include "IncrementalASTConsumer.h"

#include "clang/AST/Decl.h"
#include "clang/AST/DeclGroup.h"
#include "llvm/Support/Casting.h"

namespace clang {

bool IncrementalASTConsumer::HandleTopLevelDecl(DeclGroupRef DGR) {
  if (DGR.isNull())
    return true;
  if (!Consumer)
    return true;

  // We should append the Decl into the original DGR
  for (Decl *D : DGR)
    if (auto *TSD = llvm::dyn_cast<TopLevelStmtDecl>(D))
      if (TSD->isValuePrinting())
        if (auto *E = llvm::dyn_cast<Expr>(TSD->getStmt()))
          TSD->setStmt(Interp.SynthesizeValueGetter(E));

  return Consumer->HandleTopLevelDecl(DGR);
}

void IncrementalASTConsumer::HandleTranslationUnit(ASTContext &Context) {
  if (Consumer)
    Consumer->HandleTranslationUnit(Context);
}

} // end namespace clang
