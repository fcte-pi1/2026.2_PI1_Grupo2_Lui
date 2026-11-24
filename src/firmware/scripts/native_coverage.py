# O driver seleciona o runtime correto: LLVM no macOS, gcov com GCC.
Import("env")
env.Append(LINKFLAGS=["--coverage"])
