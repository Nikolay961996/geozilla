using System.Runtime.InteropServices;

namespace geozilla_bl.Services.Generation.Concrete
{
    internal partial class GeozillaCoreDll
    {
#if DEBUG
        private const string ConfigName = "Debug";
#else
        private const string ConfigName = "Release";
#endif

        private const string DllName = $"../../../../geozilla-core/x64/{ConfigName}/geozilla-core.dll";

        [LibraryImport(DllName, EntryPoint = "?GenerateGeoJson@@YAPEBDPEBD@Z")]
        [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
        public static partial IntPtr GenerateGeoJson([MarshalAs(UnmanagedType.LPStr)] string path);

        [LibraryImport(DllName, EntryPoint = "?FreeBuffer@@YAXPEBD@Z")]
        [UnmanagedCallConv(CallConvs = [typeof(System.Runtime.CompilerServices.CallConvCdecl)])]
        public static partial void FreeBuffer(IntPtr buffer);
    }
}
