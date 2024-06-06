using B3dmCore;

var inputfile = @"D:\code_for_me\LCT\Data\Hack-a-ton\FGM_HACKATON\Tile_p3646_p720\Tile_p3646_p720.b3dm";
Console.WriteLine(File.Exists(inputfile));

var f = File.OpenRead(inputfile);

Console.WriteLine(f.Length);

var b3dm = B3dmReader.ReadB3dm(f);
Console.WriteLine("1 - " + b3dm.FeatureTableJson.ToString());
Console.WriteLine("2 - " + string.Join(',', b3dm.FeatureTableBinary));
Console.WriteLine("3 - " + string.Join(',', b3dm.BatchTableBinary));

Console.WriteLine("= 1 - " + string.Join(',', b3dm.B3dmHeader.Validate()));
Console.WriteLine("= 2 - " + string.Join(',', b3dm.B3dmHeader.AsBinary()));
Console.WriteLine("= 3 - " + string.Join(',', b3dm.B3dmHeader.FeatureTableBinaryByteLength));
Console.WriteLine("= 4 - " + string.Join(',', b3dm.B3dmHeader.FeatureTableJsonByteLength));
Console.WriteLine("= 5 - " + string.Join(',', b3dm.B3dmHeader.BatchTableBinaryByteLength));
Console.WriteLine("= 6 - " + string.Join(',', b3dm.B3dmHeader.BatchTableJsonByteLength));
Console.WriteLine("= 7 - " + string.Join(',', b3dm.B3dmHeader.ByteLength));
Console.WriteLine("= 8 - " + string.Join(',', b3dm.B3dmHeader.Length));
Console.WriteLine("= 9 - " + string.Join(',', b3dm.B3dmHeader.Magic));
Console.WriteLine("= 10 - " + string.Join(',', b3dm.B3dmHeader.Version));


Console.WriteLine("999 - " + string.Join(',', b3dm.GlbData));



var stream = new MemoryStream(b3dm.GlbData);
Console.WriteLine(stream.ToArray());

