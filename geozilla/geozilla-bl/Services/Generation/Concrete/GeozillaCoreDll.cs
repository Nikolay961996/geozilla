using System.Runtime.InteropServices;

namespace geozilla_bl.Services.Generation.Concrete
{
    internal partial class GeozillaCoreDll
    {
#if X86
        private const string PlatformName = "x86";
#else
        private const string PlatformName = "x64";
#endif

#if DEBUG
        private const string ConfigName = "Debug";
#else
        private const string ConfigName = "Release";
#endif

        private const string DllName = $"../../../../geozilla-core/{PlatformName}/{ConfigName}/geozilla-core.dll";

        [LibraryImport(DllName, EntryPoint = "?GenerateGeoJson@@YAPEBDPEBD@Z")]
        [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
        public static partial IntPtr GenerateGeoJson([MarshalAs(UnmanagedType.LPStr)] string path);

        [LibraryImport(DllName, EntryPoint = "?FreeBuffer@@YAXPEBD@Z")]
        [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
        public static partial void FreeBuffer(IntPtr buffer);
    }
}
