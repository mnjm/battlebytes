
for %%m in (flash flash_8.22 sram) do (
    
    rmdir /Q /S %%m\settings

    for %%c in (cm0plus cm0plus_cm7_0 cm0plus_cm7_0_cm7_1 cm7_0_mc cm7_1_mc) do (

        rmdir /Q /S %%m\%%c\rev_a
        rmdir /Q /S %%m\%%c\Release
        rmdir /Q /S %%m\%%c\settings
        rmdir /Q /S %%m\%%c\libcrypto\Obj
        rmdir /Q /S %%m\%%c\libcrypto\settings
        del   /Q    %%m\%%c\BuildLog.log
        del   /Q    %%m\%%c\*.dep
        del   /Q    %%m\%%c\libcrypto\BuildLog.log
        
    )
)

