using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;

namespace geozilla_bl.Services.Generation.Concrete
{
    internal class GeozillaCoreDll
    {
#if DEBUG
        private const string ConfigName = "Debug";
#else
        private const string ConfigName = "Release";
#endif

        private const string DllName = "geozilla-core.dll";

        [DefaultDllImportSearchPaths(DllImportSearchPath.ApplicationDirectory | DllImportSearchPath.System32)]
        [DllImport(DllName, EntryPoint = "?GenerateGeoJson@@YAPEBDPEBD@Z", CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr GenerateGeoJson([MarshalAs(UnmanagedType.LPStr)] string path);

        [DefaultDllImportSearchPaths(DllImportSearchPath.ApplicationDirectory | DllImportSearchPath.System32)]
        [DllImport(DllName, EntryPoint = "?FreeBuffer@@YAXPEBD@Z", CallingConvention = CallingConvention.Cdecl)]
        public static extern void FreeBuffer(IntPtr buffer);
    }
}
