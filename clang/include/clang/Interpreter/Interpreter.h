//===--- Interpreter.h - Incremental Compilation and Execution---*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the component which performs incremental code
// compilation and execution.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_INTERPRETER_INTERPRETER_H
#define LLVM_CLANG_INTERPRETER_INTERPRETER_H

#include "clang/AST/Decl.h"
#include "clang/AST/GlobalDecl.h"
#include "clang/Interpreter/PartialTranslationUnit.h"
#include "clang/Interpreter/Value.h"

#include "llvm/ADT/DenseMap.h"
#include "llvm/ExecutionEngine/JITSymbol.h"
#include "llvm/Support/Error.h"

#include <memory>
#include <vector>

namespace llvm {
namespace orc {
class LLJIT;
class ThreadSafeContext;
} // namespace orc
} // namespace llvm

namespace clang {

class Value;
class CompilerInstance;
class IncrementalExecutor;
class IncrementalParser;

/// Create a pre-configured \c CompilerInstance for incremental processing.
class IncrementalCompilerBuilder {
public:
  static llvm::Expected<std::unique_ptr<CompilerInstance>>
  create(std::vector<const char *> &ClangArgv);
};

/// Provides top-level interfaces for incremental compilation and execution.
class Interpreter {
  std::unique_ptr<llvm::orc::ThreadSafeContext> TSCtx;
  std::unique_ptr<IncrementalParser> IncrParser;
  std::unique_ptr<IncrementalExecutor> IncrExecutor;

  Interpreter(std::unique_ptr<CompilerInstance> CI, llvm::Error &Err);

  llvm::Error CreateExecutor();
  unsigned InitPTUSize = 0;
  Value LastValue;

public:
  ~Interpreter();
  static llvm::Expected<std::unique_ptr<Interpreter>>
  create(std::unique_ptr<CompilerInstance> CI);
  ASTContext &getASTContext() const;
  const CompilerInstance *getCompilerInstance() const;
  llvm::Expected<llvm::orc::LLJIT &> getExecutionEngine();

  llvm::Expected<PartialTranslationUnit &> Parse(llvm::StringRef Code);
  llvm::Error ExecuteModule(std::unique_ptr<llvm::Module> &M);
  llvm::Error Execute(PartialTranslationUnit &T);
  llvm::Error ParseAndExecute(llvm::StringRef Code, Value *V = nullptr);
  llvm::Expected<void *> CompileDtorCall(const RecordDecl *RD);

  /// Undo N previous incremental inputs.
  llvm::Error Undo(unsigned N = 1);

  /// Link a dynamic library
  llvm::Error LoadDynamicLibrary(const char *name);

  /// \returns the \c JITTargetAddress of a \c GlobalDecl. This interface uses
  /// the CodeGenModule's internal mangling cache to avoid recomputing the
  /// mangled name.
  llvm::Expected<llvm::JITTargetAddress> getSymbolAddress(GlobalDecl GD) const;

  /// \returns the \c JITTargetAddress of a given name as written in the IR.
  llvm::Expected<llvm::JITTargetAddress>
  getSymbolAddress(llvm::StringRef IRName) const;

  /// \returns the \c JITTargetAddress of a given name as written in the object
  /// file.
  llvm::Expected<llvm::JITTargetAddress>
  getSymbolAddressFromLinkerName(llvm::StringRef LinkerName) const;

  /// Compiles the synthesized Decl and returns the JITTargetAddress.
  llvm::Expected<llvm::JITTargetAddress> CompileDecl(Decl *D);

  std::string CreateUniqName(std::string Base);

  std::list<PartialTranslationUnit> &getPTUs();

  enum InterfaceKind { NoAlloc, WithAlloc, CopyArray };

  llvm::SmallVectorImpl<Expr *> &getValuePrintingInfo() {
    return ValuePrintingInfo;
  }

  Expr *SynthesizeValueGetter(clang::Expr *E);

private:
  bool FindRuntimeInterface();

  std::unique_ptr<llvm::Module> GenModule();

  llvm::DenseMap<const RecordDecl *, void *> Dtors;

  llvm::SmallVector<Expr *, 3> ValuePrintingInfo;
};
} // namespace clang

#endif // LLVM_CLANG_INTERPRETER_INTERPRETER_H
