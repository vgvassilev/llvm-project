//===--- IncrementalExecutor.h - Incremental AST Consumer ----------*- C++
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

#ifndef LLVM_CLANG_LIB_INTERPRETER_INCREMENTALASTCONSUMER_H
#define LLVM_CLANG_LIB_INTERPRETER_INCREMENTALASTCONSUMER_H

#include "clang/AST/ASTConsumer.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/DeclGroup.h"
#include "clang/Interpreter/Interpreter.h"
#include <memory>

namespace clang {

class IncrementalASTConsumer final : public ASTConsumer {
  Interpreter &Interp;
  std::unique_ptr<ASTConsumer> Consumer;

public:
  IncrementalASTConsumer(Interpreter &InterpRef, std::unique_ptr<ASTConsumer> C)
      : Interp(InterpRef), Consumer(std::move(C)) {}

  bool HandleTopLevelDecl(DeclGroupRef DGR) override final;
  void HandleTranslationUnit(ASTContext &Context) override final;
  static bool classof(const clang::ASTConsumer *) { return true; }
};
} // end namespace clang

#endif // LLVM_CLANG_LIB_INTERPRETER_INCREMENTALEXECUTOR_H
