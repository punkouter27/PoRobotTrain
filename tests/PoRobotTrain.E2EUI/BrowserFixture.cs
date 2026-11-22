using Microsoft.Playwright;

namespace PoRobotTrain.E2EUI;

/// <summary>One Chromium for the whole run. BASE_URL points at the published Release app (scripts/run-e2eui.ps1).</summary>
public sealed class BrowserFixture : IAsyncLifetime
{
    public static string BaseUrl => (Environment.GetEnvironmentVariable("BASE_URL") ?? "http://127.0.0.1:5080").TrimEnd('/');

    private IPlaywright? _playwright;

    public IBrowser Browser { get; private set; } = null!;

    public async Task InitializeAsync()
    {
        _playwright = await Playwright.CreateAsync();
        Browser = await _playwright.Chromium.LaunchAsync(new() { Headless = Environment.GetEnvironmentVariable("HEADLESS") != "0" });
    }

    /// <summary>A page with the first-run tour marked done (and Advanced on when asked), so it never covers the page.
    /// Page errors and any browser dialog (alert/confirm) are collected; tests assert there are none.</summary>
    public async Task<(IBrowserContext Context, IPage Page, List<string> Errors)> OpenAsync(int width, int height, bool advanced = false)
    {
        var ctx = await Browser.NewContextAsync(new() { ViewportSize = new() { Width = width, Height = height } });
        var adv = advanced ? "1" : "0";
        await ctx.AddInitScriptAsync("try { localStorage.setItem('po.tour', 'done'); localStorage.setItem('po.advanced', '" + adv + "'); } catch (e) {}");
        var page = await ctx.NewPageAsync();
        var errors = new List<string>();
        page.PageError += (_, e) => errors.Add(e);
        page.Dialog += async (_, d) =>
        {
            errors.Add("dialog: " + d.Message);
            await d.DismissAsync();
        };
        return (ctx, page, errors);
    }

    public async Task DisposeAsync()
    {
        await Browser.CloseAsync();
        _playwright?.Dispose();
    }
}

[CollectionDefinition("browser")]
public sealed class BrowserTestGroup : ICollectionFixture<BrowserFixture>;
