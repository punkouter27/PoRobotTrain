using Deque.AxeCore.Commons;
using Deque.AxeCore.Playwright;
using Microsoft.Playwright;

namespace PoRobotTrain.E2EUI;

[Collection("browser")]
public sealed class EveryRouteTests(BrowserFixture fx)
{
    private const string MetricsScript = """
        () => {
          const r = s => { const e = document.querySelector(s); if (!e) return null; const b = e.getBoundingClientRect();
                           return { x: b.x, y: b.y + scrollY, right: b.right, bottom: b.bottom + scrollY }; };
          const raw = [...document.querySelectorAll('button:not(.rz-button), input:not([type=hidden]), select, textarea')]
            .filter(e => !e.closest('.rz-slider, .rz-dropdown, .rz-numeric, .rz-chkbox, .rz-fileupload') && !e.classList.contains('rz-textbox'))
            .map(e => e.outerHTML.slice(0, 80));
          const stamp = document.querySelector('.po-stamp');
          return { title: r('.po-title'), stamp: r('.po-stamp'), main: r('.po-main'), stampText: stamp ? stamp.textContent : '',
                   scrollWidth: document.documentElement.scrollWidth, clientWidth: document.documentElement.clientWidth, raw: raw.join(' | ') };
        }
        """;

    public static TheoryData<string, int> Routes()
    {
        var data = new TheoryData<string, int>();
        foreach (var route in new[] { "/", "/motions", "/walk", "/robot-health", "/setup", "/diag", "/robustness", "/no-such-page" })
        {
            data.Add(route, 1366);
            data.Add(route, 390);
        }
        return data;
    }

    [Theory]
    [MemberData(nameof(Routes))]
    public async Task RouteHasChromeNoOverlapNoRawControlsAndPassesContrast(string route, int width)
    {
        var (ctx, page, errors) = await fx.OpenAsync(width, width > 500 ? 800 : 844, advanced: true);
        try
        {
            await page.GotoAsync(BrowserFixture.BaseUrl + route, new() { WaitUntil = WaitUntilState.NetworkIdle });
            await page.Locator(".po-main").WaitForAsync();
            await page.WaitForTimeoutAsync(1500);

            var m = await page.EvaluateAsync<Metrics>(MetricsScript);

            Assert.NotNull(m.Title);
            Assert.True(m.Title!.X < width / 3.0 && m.Title.Y < 60, $"title not top-left: {m.Title.X},{m.Title.Y}");
            Assert.Matches(@"^v\d{6}$", m.StampText.Trim());
            Assert.True(m.Stamp!.Right > width - 80, $"stamp not right-aligned: right={m.Stamp.Right}");
            Assert.True(m.Stamp.Y >= m.Main!.Bottom - 1, $"stamp ({m.Stamp.Y}) is not after the content (main bottom {m.Main.Bottom})");
            Assert.True(m.ScrollWidth <= m.ClientWidth, $"sideways scroll: {m.ScrollWidth} > {m.ClientWidth}");
            Assert.True(m.Raw.Length == 0, "raw form controls (use Radzen): " + m.Raw);

            var axe = await page.RunAxe(new AxeRunOptions { RunOnly = new RunOnlyOptions { Type = "rule", Values = ["color-contrast"] } });
            Assert.True(axe.Violations.Length == 0,
                string.Join("; ", axe.Violations.SelectMany(v => v.Nodes.Select(n => string.Join(" ", n.Target) + " " + n.Any.FirstOrDefault()?.Message))));
            Assert.Empty(errors);
        }
        finally
        {
            await ctx.CloseAsync();
        }
    }

    public sealed class Box
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Right { get; set; }
        public double Bottom { get; set; }
    }

    public sealed class Metrics
    {
        public Box? Title { get; set; }
        public Box? Stamp { get; set; }
        public Box? Main { get; set; }
        public string StampText { get; set; } = "";
        public double ScrollWidth { get; set; }
        public double ClientWidth { get; set; }
        public string Raw { get; set; } = "";
    }
}
