//===------ Interpreter.cpp - Incremental Compilation and Execution -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the component which performs incremental code
// compilation and execution.
//
//===----------------------------------------------------------------------===//

#include "clang/Interpreter/Interpreter.h"

#include "IncrementalExecutor.h"
#include "IncrementalParser.h"

#include "InterpreterUtils.h"
#include "clang/AST/ASTContext.h"
#include "clang/AST/Mangle.h"
#include "clang/AST/TypeVisitor.h"
#include "clang/Basic/DiagnosticSema.h"
#include "clang/Basic/TargetInfo.h"
#include "clang/CodeGen/ModuleBuilder.h"
#include "clang/CodeGen/ObjectFilePCHContainerOperations.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/Driver.h"
#include "clang/Driver/Job.h"
#include "clang/Driver/Options.h"
#include "clang/Driver/Tool.h"
#include "clang/Frontend/CompilerInstance.h"
#include "clang/Frontend/TextDiagnosticBuffer.h"
#include "clang/Interpreter/Value.h"
#include "clang/Lex/PreprocessorOptions.h"
#include "clang/Sema/Lookup.h"
#include "llvm/ExecutionEngine/Orc/LLJIT.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Errc.h"
#include "llvm/TargetParser/Host.h"
using namespace clang;

// FIXME: Figure out how to unify with namespace init_convenience from
//        tools/clang-import-test/clang-import-test.cpp
namespace {
/// Retrieves the clang CC1 specific flags out of the compilation's jobs.
/// \returns NULL on error.
static llvm::Expected<const llvm::opt::ArgStringList *>
GetCC1Arguments(DiagnosticsEngine *Diagnostics,
                driver::Compilation *Compilation) {
  // We expect to get back exactly one Command job, if we didn't something
  // failed. Extract that job from the Compilation.
  const driver::JobList &Jobs = Compilation->getJobs();
  if (!Jobs.size() || !isa<driver::Command>(*Jobs.begin()))
    return llvm::createStringError(llvm::errc::not_supported,
                                   "Driver initialization failed. "
                                   "Unable to create a driver job");

  // The one job we find should be to invoke clang again.
  const driver::Command *Cmd = cast<driver::Command>(&(*Jobs.begin()));
  if (llvm::StringRef(Cmd->getCreator().getName()) != "clang")
    return llvm::createStringError(llvm::errc::not_supported,
                                   "Driver initialization failed");

  return &Cmd->getArguments();
}

static llvm::Expected<std::unique_ptr<CompilerInstance>>
CreateCI(const llvm::opt::ArgStringList &Argv) {
  std::unique_ptr<CompilerInstance> Clang(new CompilerInstance());
  IntrusiveRefCntPtr<DiagnosticIDs> DiagID(new DiagnosticIDs());

  // Register the support for object-file-wrapped Clang modules.
  // FIXME: Clang should register these container operations automatically.
  auto PCHOps = Clang->getPCHContainerOperations();
  PCHOps->registerWriter(std::make_unique<ObjectFilePCHContainerWriter>());
  PCHOps->registerReader(std::make_unique<ObjectFilePCHContainerReader>());

  // Buffer diagnostics from argument parsing so that we can output them using
  // a well formed diagnostic object.
  IntrusiveRefCntPtr<DiagnosticOptions> DiagOpts = new DiagnosticOptions();
  TextDiagnosticBuffer *DiagsBuffer = new TextDiagnosticBuffer;
  DiagnosticsEngine Diags(DiagID, &*DiagOpts, DiagsBuffer);
  bool Success = CompilerInvocation::CreateFromArgs(
      Clang->getInvocation(), llvm::ArrayRef(Argv.begin(), Argv.size()), Diags);

  // Infer the builtin include path if unspecified.
  if (Clang->getHeaderSearchOpts().UseBuiltinIncludes &&
      Clang->getHeaderSearchOpts().ResourceDir.empty())
    Clang->getHeaderSearchOpts().ResourceDir =
        CompilerInvocation::GetResourcesPath(Argv[0], nullptr);

  // Create the actual diagnostics engine.
  Clang->createDiagnostics();
  if (!Clang->hasDiagnostics())
    return llvm::createStringError(llvm::errc::not_supported,
                                   "Initialization failed. "
                                   "Unable to create diagnostics engine");

  DiagsBuffer->FlushDiagnostics(Clang->getDiagnostics());
  if (!Success)
    return llvm::createStringError(llvm::errc::not_supported,
                                   "Initialization failed. "
                                   "Unable to flush diagnostics");

  // FIXME: Merge with CompilerInstance::ExecuteAction.
  llvm::MemoryBuffer *MB = llvm::MemoryBuffer::getMemBuffer("").release();
  Clang->getPreprocessorOpts().addRemappedFile("<<< inputs >>>", MB);

  Clang->setTarget(TargetInfo::CreateTargetInfo(
      Clang->getDiagnostics(), Clang->getInvocation().TargetOpts));
  if (!Clang->hasTarget())
    return llvm::createStringError(llvm::errc::not_supported,
                                   "Initialization failed. "
                                   "Target is missing");

  Clang->getTarget().adjust(Clang->getDiagnostics(), Clang->getLangOpts());

  // Don't clear the AST before backend codegen since we do codegen multiple
  // times, reusing the same AST.
  Clang->getCodeGenOpts().ClearASTBeforeBackend = false;

  Clang->getFrontendOpts().DisableFree = false;
  Clang->getCodeGenOpts().DisableFree = false;

  return std::move(Clang);
}

} // anonymous namespace

llvm::Expected<std::unique_ptr<CompilerInstance>>
IncrementalCompilerBuilder::create(std::vector<const char *> &ClangArgv) {

  // If we don't know ClangArgv0 or the address of main() at this point, try
  // to guess it anyway (it's possible on some platforms).
  std::string MainExecutableName =
      llvm::sys::fs::getMainExecutable(nullptr, nullptr);

  ClangArgv.insert(ClangArgv.begin(), MainExecutableName.c_str());

  // Prepending -c to force the driver to do something if no action was
  // specified. By prepending we allow users to override the default
  // action and use other actions in incremental mode.
  // FIXME: Print proper driver diagnostics if the driver flags are wrong.
  // We do C++ by default; append right after argv[0] if no "-x" given
  ClangArgv.insert(ClangArgv.end(), "-xc++");
  ClangArgv.insert(ClangArgv.end(), "-Xclang");
  ClangArgv.insert(ClangArgv.end(), "-fincremental-extensions");
  ClangArgv.insert(ClangArgv.end(), "-c");

  // Put a dummy C++ file on to ensure there's at least one compile job for the
  // driver to construct.
  ClangArgv.push_back("<<< inputs >>>");

  // Buffer diagnostics from argument parsing so that we can output them using a
  // well formed diagnostic object.
  IntrusiveRefCntPtr<DiagnosticIDs> DiagID(new DiagnosticIDs());
  IntrusiveRefCntPtr<DiagnosticOptions> DiagOpts =
      CreateAndPopulateDiagOpts(ClangArgv);
  TextDiagnosticBuffer *DiagsBuffer = new TextDiagnosticBuffer;
  DiagnosticsEngine Diags(DiagID, &*DiagOpts, DiagsBuffer);

  driver::Driver Driver(/*MainBinaryName=*/ClangArgv[0],
                        llvm::sys::getProcessTriple(), Diags);
  Driver.setCheckInputsExist(false); // the input comes from mem buffers
  llvm::ArrayRef<const char *> RF = llvm::ArrayRef(ClangArgv);
  std::unique_ptr<driver::Compilation> Compilation(Driver.BuildCompilation(RF));

  if (Compilation->getArgs().hasArg(driver::options::OPT_v))
    Compilation->getJobs().Print(llvm::errs(), "\n", /*Quote=*/false);

  auto ErrOrCC1Args = GetCC1Arguments(&Diags, Compilation.get());
  if (auto Err = ErrOrCC1Args.takeError())
    return std::move(Err);

  return CreateCI(**ErrOrCC1Args);
}

Interpreter::Interpreter(std::unique_ptr<CompilerInstance> CI,
                         llvm::Error &Err) {
  llvm::ErrorAsOutParameter EAO(&Err);
  auto LLVMCtx = std::make_unique<llvm::LLVMContext>();
  TSCtx = std::make_unique<llvm::orc::ThreadSafeContext>(std::move(LLVMCtx));
  IncrParser = std::make_unique<IncrementalParser>(*this, std::move(CI),
                                                   *TSCtx->getContext(), Err);
}

Interpreter::~Interpreter() {
  if (IncrExecutor) {
    if (llvm::Error Err = IncrExecutor->cleanUp())
      llvm::report_fatal_error(
          llvm::Twine("Failed to clean up IncrementalExecutor: ") +
          toString(std::move(Err)));
  }
}

// These better to put in a runtime header but we can't. This is because we
// can't find the percise resource directory in unittests so we have to hard
// code them.
const char *const Runtimes = R"(
    #include <new>
//    namespace clang { class Value; }
//    externclang::Value* gValue = nullptr;
    void *__InterpreterSetValueWithAlloc(void*, void*, void*);
    void __InterpreterSetValueNoAlloc(void*, void*, void*, void*);
    void __InterpreterSetValueNoAlloc(void*, void*, void*, float);
    void __InterpreterSetValueNoAlloc(void*, void*, void*, double);
    void __InterpreterSetValueNoAlloc(void*, void*, void*, long double);
    void __InterpreterSetValueNoAlloc(void*,void*,void*,unsigned long long);
    template <class T, class = T (*)() /*disable for arrays*/>
    void __InterpreterSetValueCopyArr(T* Src, void* Placement, std::size_t Size) {
      for (auto Idx = 0; Idx < Size; ++Idx)
        new ((void*)(((T*)Placement) + Idx)) T(Src[Idx]);
    }
    template <class T, std::size_t N>
    void __InterpreterSetValueCopyArr(const T (*Src)[N], void* Placement, std::size_t Size) {
      __InterpreterSetValueCopyArr(Src[0], Placement, Size);
    }
)";

llvm::Expected<std::unique_ptr<Interpreter>>
Interpreter::create(std::unique_ptr<CompilerInstance> CI) {
  llvm::Error Err = llvm::Error::success();
  auto Interp =
      std::unique_ptr<Interpreter>(new Interpreter(std::move(CI), Err));
  if (Err)
    return std::move(Err);
  if (llvm::Error Err = Interp->ParseAndExecute(Runtimes))
    return std::move(Err);

  Interp->ValuePrintingInfo.resize(3);
  // FIXME: This is a ugly hack. Undo command checks its availability by looking
  // at the size of the PTU list. However we have parsed something in the
  // beginning of the REPL so we have to mark them as 'Irrevocable'.
  Interp->InitPTUSize = Interp->getPTUs().size();
  return std::move(Interp);
}

const CompilerInstance *Interpreter::getCompilerInstance() const {
  return IncrParser->getCI();
}

llvm::Expected<llvm::orc::LLJIT &> Interpreter::getExecutionEngine() {
  if (!IncrExecutor) {
    if (auto Err = CreateExecutor())
      return std::move(Err);
  }

  return IncrExecutor->GetExecutionEngine();
}

ASTContext &Interpreter::getASTContext() const {
  return getCompilerInstance()->getASTContext();
}

std::list<PartialTranslationUnit> &Interpreter::getPTUs() {
  return IncrParser->getPTUs();
}

std::unique_ptr<llvm::Module> Interpreter::GenModule() {
  return IncrParser->GenModule();
}

std::string Interpreter::CreateUniqName(std::string Base) {
  static size_t I = 0;
  Base += std::to_string(I);
  I += 1;
  return Base;
}

llvm::Expected<PartialTranslationUnit &>
Interpreter::Parse(llvm::StringRef Code) {
  // Tell the interpreter sliently ignore unused expressions since value
  // printing could cause it.
  getCompilerInstance()->getDiagnostics().setSeverity(
      clang::diag::warn_unused_expr, diag::Severity::Ignored, SourceLocation());

  return IncrParser->Parse(Code);
}

llvm::Error Interpreter::CreateExecutor() {
  const clang::TargetInfo &TI =
      getCompilerInstance()->getASTContext().getTargetInfo();
  llvm::Error Err = llvm::Error::success();
  auto Executor = std::make_unique<IncrementalExecutor>(*TSCtx, Err, TI);
  if (!Err)
    IncrExecutor = std::move(Executor);

  return Err;
}

llvm::Error Interpreter::ExecuteModule(std::unique_ptr<llvm::Module> &M) {
  if (!IncrExecutor) {
    auto Err = CreateExecutor();
    if (Err)
      return Err;
  }
  // FIXME: Add a callback to retain the llvm::Module once the JIT is done.
  if (auto Err = IncrExecutor->addModule(M))
    return Err;

  if (auto Err = IncrExecutor->runCtors())
    return Err;

  return llvm::Error::success();
}

llvm::Error Interpreter::Execute(PartialTranslationUnit &PTU) {
  if (!IncrExecutor) {
    auto Err = CreateExecutor();
    if (Err)
      return Err;
  }
  // FIXME: Add a callback to retain the llvm::Module once the JIT is done.
  if (auto Err = IncrExecutor->addModule(PTU))
    return Err;

  if (auto Err = IncrExecutor->runCtors())
    return Err;

  return llvm::Error::success();
}

llvm::Error Interpreter::ParseAndExecute(llvm::StringRef Code, Value *V) {
  auto PTU = Parse(Code);
  if (!PTU)
    return PTU.takeError();
  if (PTU->TheModule)
    if (llvm::Error Err = Execute(*PTU))
      return Err;

  // if (auto *Main = llvm::jitTargetAddressToPointer<void (*)(void *, void *)>(
  //         AddrOrErr.value())) {
  //   if (V) {
  //     (*Main)((void *)this, (void *)V);
  //     return llvm::Error::success();
  //   }
  //   Value DefaultValue;
  //   (*Main)((void *)this, (void *)&DefaultValue);
  //   DefaultValue.dump();
  //   return llvm::Error::success();
  // }

  return llvm::Error::success();
}

llvm::Expected<llvm::JITTargetAddress>
Interpreter::getSymbolAddress(GlobalDecl GD) const {
  if (!IncrExecutor)
    return llvm::make_error<llvm::StringError>("Operation failed. "
                                               "No execution engine",
                                               std::error_code());
  llvm::StringRef MangledName = IncrParser->GetMangledName(GD);
  return getSymbolAddress(MangledName);
}

llvm::Expected<llvm::JITTargetAddress>
Interpreter::getSymbolAddress(llvm::StringRef IRName) const {
  if (!IncrExecutor)
    return llvm::make_error<llvm::StringError>("Operation failed. "
                                               "No execution engine",
                                               std::error_code());

  return IncrExecutor->getSymbolAddress(IRName, IncrementalExecutor::IRName);
}

llvm::Expected<llvm::JITTargetAddress>
Interpreter::getSymbolAddressFromLinkerName(llvm::StringRef Name) const {
  if (!IncrExecutor)
    return llvm::make_error<llvm::StringError>("Operation failed. "
                                               "No execution engine",
                                               std::error_code());

  return IncrExecutor->getSymbolAddress(Name, IncrementalExecutor::LinkerName);
}

llvm::Error Interpreter::Undo(unsigned N) {

  std::list<PartialTranslationUnit> &PTUs = IncrParser->getPTUs();
  if (N + InitPTUSize > PTUs.size())
    return llvm::make_error<llvm::StringError>("Operation failed. "
                                               "Too many undos",
                                               std::error_code());
  for (unsigned I = 0; I < N; I++) {
    if (IncrExecutor) {
      if (llvm::Error Err = IncrExecutor->removeModule(PTUs.back()))
        return Err;
    }

    IncrParser->CleanUpPTU(PTUs.back());
    PTUs.pop_back();
  }
  return llvm::Error::success();
}

llvm::Error Interpreter::LoadDynamicLibrary(const char *name) {
  auto EE = getExecutionEngine();
  if (!EE)
    return EE.takeError();

  auto &DL = EE->getDataLayout();

  if (auto DLSG = llvm::orc::DynamicLibrarySearchGenerator::Load(
          name, DL.getGlobalPrefix()))
    EE->getMainJITDylib().addGenerator(std::move(*DLSG));
  else
    return DLSG.takeError();

  return llvm::Error::success();
}

llvm::Expected<llvm::JITTargetAddress> Interpreter::CompileDecl(Decl *D) {
  assert(D && "The Decl being compiled can't be null");

  ASTConsumer &Consumer = getCompilerInstance()->getASTConsumer();
  Consumer.HandleTopLevelDecl(DeclGroupRef(D));
  getCompilerInstance()->getSema().PerformPendingInstantiations();
  Consumer.HandleTranslationUnit(getASTContext());

  if (std::unique_ptr<llvm::Module> M = GenModule()) {
    if (llvm::Error Err = ExecuteModule(M))
      return Err;
    ASTNameGenerator ASTNameGen(getASTContext());
    llvm::Expected<llvm::JITTargetAddress> AddrOrErr =
        getSymbolAddressFromLinkerName(ASTNameGen.getName(D));

    return AddrOrErr;
  }
  return llvm::JITTargetAddress{};
}

llvm::Expected<void *> Interpreter::CompileDtorCall(const RecordDecl *RD) {
  if (auto Dtor = Dtors.find(RD); Dtor != Dtors.end())
    return Dtor->getSecond();

  // No need to generate the destructor if it has no semantic effects.
  if (const CXXRecordDecl *CXXRD = llvm::dyn_cast<CXXRecordDecl>(RD);
      CXXRD->hasIrrelevantDestructor())
    return nullptr;

  // Generate AST.
  //
  // extern "C" void __InterpreterDtorT(void* obj) {
  //   ((T*)obj)->~T();
  // }
  //
  QualType QT(RD->getTypeForDecl(), 0);

  std::string DtorName;
  llvm::raw_string_ostream SS(DtorName);
  SS << "__InterpreterDtor" << RD;

  std::string TypeName = GetFullTypeName(getASTContext(), QT);

  const char *CodeTemplate = R"(
    extern "C" void {0}(void* obj) {
      (({1}*)obj)->~{2}();
    }
  )";
  std::string Code =
      llvm::formatv(CodeTemplate, DtorName, TypeName, RD->getName());
  llvm::Expected<PartialTranslationUnit &> PTUOrErr = Parse(Code);
  // FIXME: Can't we just synthesize the AST directly?
#if 0
  Sema &S = getCompilerInstance()->getSema();
  ASTContext &Ctx = S.getASTContext();

  QualType QT(RD->getTypeForDecl(), 0);
  std::string DtorName = "__InterpreterDtor" + QT.getAsString();

  QualType RetTy = Ctx.VoidTy;
  QualType ArgTy = Ctx.VoidPtrTy;

  QualType FunctionTy = Ctx.getFunctionType(RetTy, {ArgTy}, {});
  IdentifierInfo *II = &Ctx.Idents.get(DtorName);
  // Extern "C" ?
  auto *FD = FunctionDecl::Create(
      S.getASTContext(), Ctx.getTranslationUnitDecl(), SourceLocation(),
      SourceLocation(), II, FunctionTy, /*TInfo=*/nullptr, SC_Extern);
  auto *ObjParm = ParmVarDecl::Create(
      Ctx, FD, SourceLocation(), SourceLocation(), /*Id=*/nullptr, ArgTy,
      Ctx.getTrivialTypeSourceInfo(ArgTy, SourceLocation()), SC_None,
      /*DefArg=*/nullptr);
  FD->setParams({ObjParm});

  // The function Body.
  auto *Obj = DeclRefExpr::Create(Ctx, NestedNameSpecifierLoc(),
                                  SourceLocation(), ObjParm,
                                  /*RefersToEnclosingVariableOrCapture=*/false,
                                  SourceLocation(), ArgTy, VK_PRValue);
  // Force cast it to the right type.
  Expr *CastedExpr = CStyleCastPtrExpr(S, QT, Obj);

  ExprResult DtorCall = nullptr;

  assert(!DtorCall.isInvalid() && "Can't create the destructor call!");

  FD->setBody(DtorCall.get());
  // Compile it.
  llvm::Expected<PartialTranslationUnit &> PTUOrErr =
      IncrParser->Parse(CreateDGPtrFrom(S, FD));
#endif
  if (!PTUOrErr)
    return PTUOrErr.takeError();
  if (PTUOrErr->TheModule)
    if (llvm::Error Err = Execute(*PTUOrErr))
      return Err;

  // Look up the function address in the JIT.
  if (llvm::Expected<llvm::JITTargetAddress> AddrOrErr =
          getSymbolAddressFromLinkerName(DtorName)) {
    void *Dtor = (void *)AddrOrErr.get();
    Dtors[RD] = Dtor;
    return Dtor;
  }
  return llvm::make_error<llvm::StringError>("Can't compile a destructor",
                                             std::error_code());
}

static constexpr llvm::StringRef MagicRuntimeInterface[] = {
    "__InterpreterSetValueNoAlloc", "__InterpreterSetValueWithAlloc",
    "__InterpreterSetValueCopyArr"};

bool Interpreter::FindRuntimeInterface() {
  if (llvm::all_of(ValuePrintingInfo, [](Expr *E) { return E != nullptr; }))
    return true;

  Sema &S = getCompilerInstance()->getSema();
  ASTContext &Ctx = S.getASTContext();

  auto LookupInterface = [&](Expr *&Interface, llvm::StringRef Name) {
    LookupResult R(S, &Ctx.Idents.get(Name), SourceLocation(),
                   Sema::LookupOrdinaryName, Sema::ForVisibleRedeclaration);
    S.LookupQualifiedName(R, Ctx.getTranslationUnitDecl());
    if (R.empty())
      return false;

    CXXScopeSpec CSS;
    Interface = S.BuildDeclarationNameExpr(CSS, R, /*ADL=*/false).get();
    return true;
  };

  if (!LookupInterface(ValuePrintingInfo[NoAlloc],
                       MagicRuntimeInterface[NoAlloc]))
    return false;
  if (!LookupInterface(ValuePrintingInfo[WithAlloc],
                       MagicRuntimeInterface[WithAlloc]))
    return false;
  if (!LookupInterface(ValuePrintingInfo[CopyArray],
                       MagicRuntimeInterface[CopyArray]))
    return false;
  return true;
}

namespace {

class RuntimeInterfaceBuilder
    : public TypeVisitor<RuntimeInterfaceBuilder, Interpreter::InterfaceKind> {
  clang::Interpreter &Interp;
  ASTContext &Ctx;
  Sema &S;
  Expr *E;
  llvm::SmallVector<Expr *, 3> Args;

public:
  RuntimeInterfaceBuilder(clang::Interpreter &In, ASTContext &C, Sema &SemaRef,
                          Expr *VE, ArrayRef<Expr *> FixedArgs)
      : Interp(In), Ctx(C), S(SemaRef), E(VE) {
    // The Interpreter* parameter and the out parameter `OutVal`.
    for (Expr *E : FixedArgs)
      Args.push_back(E);

    // Get rid of ExprWithCleanups.
    if (auto *EWC = llvm::dyn_cast_if_present<ExprWithCleanups>(E))
      E = EWC->getSubExpr();
  }

  ExprResult getCall() {
    QualType Ty = E->getType();
    QualType DesugaredTy = Ty.getDesugaredType(Ctx);

    // For lvalue struct, we treat it as a reference.
    if (DesugaredTy->isRecordType() && E->isLValue()) {
      DesugaredTy = Ctx.getLValueReferenceType(DesugaredTy);
      Ty = Ctx.getLValueReferenceType(Ty);
    }

    Expr *TypeArg =
        CStyleCastPtrExpr(S, Ctx.VoidPtrTy, (uintptr_t)Ty.getAsOpaquePtr());
    // The QualType parameter `OpaqueType`, represented as `void*`.
    Args.push_back(TypeArg);

    // We push the last parameter based on the type of the Expr. Note we need
    // special care for rvalue struct.
    Interpreter::InterfaceKind Kind = Visit(&*DesugaredTy);
    switch (Kind) {
    case Interpreter::InterfaceKind::WithAlloc:
    case Interpreter::InterfaceKind::CopyArray: {
      // __InterpreterSetValueWithAlloc(ThisInterp, OpaqueValue, OpaqueType)
      ExprResult AllocCall = S.ActOnCallExpr(
          /*Scope=*/nullptr,
          Interp.getValuePrintingInfo()[Interpreter::InterfaceKind::WithAlloc],
          E->getBeginLoc(), Args, E->getEndLoc());
      assert(!AllocCall.isInvalid() && "Can't create runtime interface call!");

      TypeSourceInfo *TSI = Ctx.getTrivialTypeSourceInfo(Ty, SourceLocation());

      // __InterpreterSetValueCopyArr(T* Src, void* Placement, std::size_t Size)
      if (Kind == Interpreter::InterfaceKind::CopyArray) {
        const auto *ConstantArrTy =
            cast<ConstantArrayType>(DesugaredTy.getTypePtr());
        size_t ArrSize = Ctx.getConstantArrayElementCount(ConstantArrTy);
        Expr *ArrSizeExpr = IntegerLiteralExpr(Ctx, ArrSize);
        Expr *Args[] = {E, AllocCall.get(), ArrSizeExpr};
        return S.ActOnCallExpr(
            /*Scope *=*/nullptr,
            Interp
                .getValuePrintingInfo()[Interpreter::InterfaceKind::CopyArray],
            SourceLocation(), Args, SourceLocation());
      }
      Expr *Args[] = {AllocCall.get()};
      ExprResult CXXNewCall = S.BuildCXXNew(
          E->getSourceRange(),
          /*UseGlobal=*/true, /*PlacementLParen=*/SourceLocation(), Args,
          /*PlacementRParen=*/SourceLocation(),
          /*TypeIdParens=*/SourceRange(), TSI->getType(), TSI, std::nullopt,
          E->getSourceRange(), E);

      assert(!CXXNewCall.isInvalid() &&
             "Can't create runtime placement new call!");

      return S.ActOnFinishFullExpr(CXXNewCall.get(),
                                   /*DiscardedValue=*/false);
    }
      // __InterpreterSetValueNoAlloc(ThisInterp, OpaqueValue, QT, E)
    case Interpreter::InterfaceKind::NoAlloc: {
      return S.ActOnCallExpr(
          /*Scope=*/nullptr,
          Interp.getValuePrintingInfo()[Interpreter::InterfaceKind::NoAlloc],
          E->getBeginLoc(), Args, E->getEndLoc());
    }
      llvm_unreachable("Unknown runtime interface kind");
    }
  }

  Interpreter::InterfaceKind VisitRecordType(const RecordType *Ty) {
    return Interpreter::InterfaceKind::WithAlloc;
  }

  Interpreter::InterfaceKind
  VisitMemberPointerType(const MemberPointerType *Ty) {
    llvm_unreachable("Not implemented yet");
  }
  Interpreter::InterfaceKind
  VisitConstantArrayType(const ConstantArrayType *Ty) {
    return Interpreter::InterfaceKind::CopyArray;
  }

  Interpreter::InterfaceKind VisitPointerType(const PointerType *Ty) {
    TypeSourceInfo *TSI = Ctx.getTrivialTypeSourceInfo(Ctx.VoidPtrTy);
    ExprResult CastedExpr =
        S.BuildCStyleCastExpr(SourceLocation(), TSI, SourceLocation(), E);
    assert(!CastedExpr.isInvalid() && "Can not create cstyle cast expression");
    Args.push_back(CastedExpr.get());
    return Interpreter::InterfaceKind::NoAlloc;
  }

  Interpreter::InterfaceKind VisitReferenceType(const ReferenceType *Ty) {
    ExprResult AddrOfE = S.CreateBuiltinUnaryOp(SourceLocation(), UO_AddrOf, E);
    assert(!AddrOfE.isInvalid() && "Can not create unary expression");
    Args.push_back(AddrOfE.get());
    return Interpreter::InterfaceKind::NoAlloc;
  }

  Interpreter::InterfaceKind VisitBuiltinType(const BuiltinType *Ty) {
    if (Ty->isNullPtrType())
      Args.push_back(E);
    else if (Ty->isFloatingType())
      Args.push_back(E);
    else if (Ty->isVoidType())
      Args.push_back(E);
    else if (Ty->isIntegralOrEnumerationType())
      HandleIntegralOrEnumType(Ty);

    return Interpreter::InterfaceKind::NoAlloc;
  }

  Interpreter::InterfaceKind VisitEnumType(const EnumType *Ty) {
    HandleIntegralOrEnumType(Ty);
    return Interpreter::InterfaceKind::NoAlloc;
  }

private:
  // Force cast these types to uint64 to reduce the number of overloads of
  // `__InterpreterSetValueNoAlloc`.
  void HandleIntegralOrEnumType(const Type *Ty) {
    TypeSourceInfo *TSI = Ctx.getTrivialTypeSourceInfo(Ctx.UnsignedLongLongTy);
    ExprResult CastedExpr =
        S.BuildCStyleCastExpr(SourceLocation(), TSI, SourceLocation(), E);
    assert(!CastedExpr.isInvalid() && "Cannot create cstyle cast expr");
    Args.push_back(CastedExpr.get());
  }
};
} // namespace

static constexpr const char *const ValueGetter = "__InterpreterValueGetter";

// This synthesizes a wrapper function that used in passing the Value object to
// the interpreter. Inside the wrapper we synthesize another call to a speciall
// function that is responsible for generating the Value.
// In general, we transform:
//   clang-repl> x
// To:
// void __InterpreterValueGetter(void* ThisInterp, void* OpaqueValue) {
//   // 1. If x is a built-in type like int, float.
//   __InterpreterSetValueNoAlloc(ThisInterp, OpaqueValue, xExpr, x);
//   // 2. If x is a struct, and a lvalue.
//   __InterpreterSetValueNoAlloc(ThisInterp, OpaqueValue, xExpr, &x);
//   // 3. If x is a struct, but a rvalue.
//   new (__InterpreterSetValueWithAlloc(ThisInterp, OpaqueValue, xExpr))
//   (x);
// }
Expr *Interpreter::SynthesizeValueGetter(clang::Expr *E) {
  Sema &S = getCompilerInstance()->getSema();
  ASTContext &Ctx = S.getASTContext();

  QualType Void = Ctx.VoidTy;
  QualType VoidPtr = Ctx.VoidPtrTy;

  // Synthesize function `__InterpreterValueGetter`.
  QualType FunctionTy = Ctx.getFunctionType(Void, {VoidPtr, VoidPtr}, {});
  IdentifierInfo *II = &Ctx.Idents.get(CreateUniqName(ValueGetter));
  auto *FD = FunctionDecl::Create(
      S.getASTContext(), Ctx.getTranslationUnitDecl(), SourceLocation(),
      SourceLocation(), II, FunctionTy, /*TInfo=*/nullptr, SC_None);

  // The two parameters:
  //   1. void* OpaqueInterp
  //   2. void* OpaqueValue
  auto *ThisInterp = CStyleCastPtrExpr(S, Ctx.VoidPtrTy, (uintptr_t)this);

  if (!FindRuntimeInterface())
    llvm_unreachable("We can't find the runtime iterface for pretty print!");

  // Create parameter `OutVal` for `__InterpreterSetValue*`
  // LookupResult R(S, &Ctx.Idents.get("gValue"), SourceLocation(),
  //                Sema::LookupOrdinaryName, Sema::ForVisibleRedeclaration);
  // S.LookupQualifiedName(R, Ctx.getTranslationUnitDecl());
  // assert(!R.empty());

  // CXXScopeSpec CSS;
  // auto *OutValue = S.BuildDeclarationNameExpr(CSS, R, /*ADL=*/false).get();
  //OutValue = S.BuildUnaryOp(/*Scope=*/nullptr, SourceLocation(), UO_AddrOf, OutValue).get();
  auto *OutValue = CStyleCastPtrExpr(S, Ctx.VoidPtrTy, (uintptr_t)&this->LastValue);

  // Build `__InterpreterSetValue*` call.
  RuntimeInterfaceBuilder Builder(*this, Ctx, S, E, {ThisInterp, OutValue});

  ExprResult Result = Builder.getCall();
  assert(!Result.isInvalid() && "Failed to generate the CallExpr!");
  FD->setBody(Result.get());
  return Result.get();
}

// Temporary rvalue struct that need special care.
REPL_EXTERNAL_VISIBILITY void *
__InterpreterSetValueWithAlloc(void *This, void *OutVal, void *OpaqueType) {
  Value &VRef = *(Value *)OutVal;
  VRef = Value(This, OpaqueType);
  return VRef.getPtr();
}

// Pointers, lvalue struct that can take as a reference.
REPL_EXTERNAL_VISIBILITY void __InterpreterSetValueNoAlloc(void *This,
                                                           void *OutVal,
                                                           void *OpaqueType,
                                                           void *Val) {
  Value &VRef = *(Value *)OutVal;
  VRef = Value(This, OpaqueType);
  VRef.setPtr(Val);
  VRef.dump();
}

REPL_EXTERNAL_VISIBILITY void
__InterpreterSetValueNoAlloc(void *This, void *OutVal, void *OpaqueType,
                             unsigned long long Val) {
  Value &VRef = *(Value *)OutVal;
  VRef = Value(This, OpaqueType);
  VRef.setULongLong(Val);
  VRef.dump();
}

REPL_EXTERNAL_VISIBILITY void __InterpreterSetValueNoAlloc(void *This,
                                                           void *OutVal,
                                                           void *OpaqueType,
                                                           float Val) {
  Value &VRef = *(Value *)OutVal;
  VRef = Value(This, OpaqueType);
  VRef.setFloat(Val);
  VRef.dump();
}

REPL_EXTERNAL_VISIBILITY void __InterpreterSetValueNoAlloc(void *This,
                                                           void *OutVal,
                                                           void *OpaqueType,
                                                           double Val) {
  Value &VRef = *(Value *)OutVal;
  VRef = Value(This, OpaqueType);
  VRef.setDouble(Val);
  VRef.dump();
}

REPL_EXTERNAL_VISIBILITY void __InterpreterSetValueNoAlloc(void *This,
                                                           void *OutVal,
                                                           void *OpaqueType,
                                                           long double Val) {
  Value &VRef = *(Value *)OutVal;
  VRef = Value(This, OpaqueType);
  VRef.setLongDouble(Val);
  VRef.dump();
}
