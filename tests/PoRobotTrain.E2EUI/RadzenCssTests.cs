using Microsoft.Playwright;

namespace PoRobotTrain.E2EUI;

[Collection("browser")]
public sealed class RadzenCssTests(BrowserFixture fx)
{
    private const string SheetsScript = """
        () => {
          const sheets = [...document.styleSheets];
          const hrefs = [];
          const walk = s => { if (s.href) hrefs.push(s.href);
                              for (const r of s.cssRules) if (r instanceof CSSImportRule && r.styleSheet) walk(r.styleSheet); };
          sheets.forEach(walk);
          const app = sheets.find(s => (s.href || '').includes('/css/app.css'));
          const rules = app ? [...app.cssRules] : [];
          const firstImport = rules.findIndex(r => r instanceof CSSImportRule);
          return {
            radzen: hrefs.filter(h => /radzen/i.test(h)).length,
            firstSheetIsApp: !!sheets[0] && (sheets[0].href || '').includes('/css/app.css'),
            layerOrder: rules[0] && rules[0].nameList ? rules[0].nameList.join(',') : '',
            importIsRadzen: firstImport === 1 && /radzen/i.test(rules[1].href) && rules[1].layerName === 'radzen',
            appRulesAfterImport: rules.length > firstImport + 1,
          };
        }
        """;

    /// <summary>
    /// Radzen's theme CSS loads exactly once, first in &lt;head&gt;: app.css (the first stylesheet) opens with
    /// <c>@layer radzen, po-reset;</c> and then imports the Radzen theme into the low <c>radzen</c> layer, before any of
    /// the app's own rules. No second copy (no RadzenTheme component, no extra link tag).
    /// </summary>
    [Fact]
    public async Task RadzenCssLoadsOnceBeforeTheAppCss()
    {
        var (ctx, page, errors) = await fx.OpenAsync(1366, 800);
        try
        {
            await page.GotoAsync(BrowserFixture.BaseUrl + "/diag", new() { WaitUntil = WaitUntilState.NetworkIdle });
            await page.Locator(".po-main").WaitForAsync();
            var m = await page.EvaluateAsync<Sheets>(SheetsScript);
            Assert.True(m.FirstSheetIsApp, "app.css must be the first stylesheet in <head>");
            Assert.Equal("radzen,po-reset", m.LayerOrder);
            Assert.True(m.ImportIsRadzen, "the Radzen theme must be the first import, in layer radzen");
            Assert.True(m.AppRulesAfterImport);
            Assert.Equal(1, m.Radzen);
            Assert.Empty(errors);
        }
        finally
        {
            await ctx.CloseAsync();
        }
    }

    public sealed class Sheets
    {
        public int Radzen { get; set; }
        public bool FirstSheetIsApp { get; set; }
        public string LayerOrder { get; set; } = "";
        public bool ImportIsRadzen { get; set; }
        public bool AppRulesAfterImport { get; set; }
    }
}
