using Microsoft.Playwright;

namespace PoRobotTrain.E2EUI;

[Collection("browser")]
public sealed class DiagTests(BrowserFixture fx)
{
    private const string MutedColorScript = """
        () => { const s = document.createElement('span'); s.style.color = 'var(--po-muted)'; document.body.append(s);
                const c = getComputedStyle(s).color; s.remove(); return c; }
        """;

    /// <summary>
    /// /diag probes live (the behaviour service through the API, plus the browser runtime). A capability that is off on
    /// purpose (WebGPU in headless Chromium, speech recognition) shows as a neutral grey OFF and does not fail the page.
    /// </summary>
    [Fact]
    public async Task DiagProbesLiveAndShowsSwitchedOffCapabilitiesAsNeutralOff()
    {
        var (ctx, page, errors) = await fx.OpenAsync(1366, 900);
        try
        {
            await page.GotoAsync(BrowserFixture.BaseUrl + "/diag", new() { WaitUntil = WaitUntilState.NetworkIdle });
            await Assertions.Expect(page.Locator("dt", new() { HasText = "Upstream Reachable" })).ToBeVisibleAsync(new() { Timeout = 20000 });
            await Assertions.Expect(page.Locator("dt:has-text('Upstream Reachable') + dd")).ToHaveTextAsync("Yes");

            var pill = page.Locator(".sec-hdr .po-pill");
            await Assertions.Expect(pill).Not.ToHaveTextAsync("RUNNING…", new() { Timeout = 60000 });

            var off = page.Locator("td.off");
            Assert.True(await off.CountAsync() > 0, "headless Chromium has no WebGPU, so at least one OFF row is expected");
            await Assertions.Expect(off.First).ToHaveTextAsync("OFF");
            var color = await off.First.EvaluateAsync<string>("e => getComputedStyle(e).color");
            Assert.Equal(await page.EvaluateAsync<string>(MutedColorScript), color);
            var failing = await page.Locator("td.bad").CountAsync();
            await Assertions.Expect(pill).ToHaveTextAsync(failing == 0 ? "ALL CHECKS PASS" : $"{failing} FAILING");
            Assert.Empty(errors);
        }
        finally
        {
            await ctx.CloseAsync();
        }
    }
}
