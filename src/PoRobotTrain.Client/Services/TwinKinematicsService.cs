using System.Net.Http.Json;
using PoRobotTrain.Shared.Kinematics;

namespace PoRobotTrain.Client.Services;

/// <summary>Loads twin/twin_manifest.json once and exposes Silk.NET forward kinematics for read-outs.</summary>
public sealed class TwinKinematicsService(HttpClient http)
{
    private Task<TwinKinematics?>? _load;

    public Task<TwinKinematics?> GetAsync() => _load ??= LoadAsync();

    private async Task<TwinKinematics?> LoadAsync()
    {
        try
        {
            var manifest = await http.GetFromJsonAsync("twin/twin_manifest.json", TwinJsonContext.Default.TwinManifestDto);
            return manifest is null ? null : new TwinKinematics(manifest);
        }
        catch (HttpRequestException)
        {
            return null;
        }
        catch (System.Text.Json.JsonException)
        {
            return null;
        }
    }
}
